// Handles the "which victims, in what order" decision on top of the
// RoadmapGraph.Functions:
//   1) buildMissionContext: snap start/victims/gate onto the graph as extra
//      nodes, run Dijkstra from each of them, assemble the POI distance
//      matrix D.
//   2) solveMissionWithBudget: feed D + victim values + a distance budget to
//      the Orienteering solver (orienteering.cpp). Then stitch the winning
//      victims' individual shortest paths into one continuous graph_path.
//   3) computeVictimMission: convenience wrapper that also derives the
//      distance budget from the mission's time limit.
#include "task/victim_mission.hpp"

#include "task/orienteering.hpp"
#include "utils/roadmap_utils.hpp"
#include "utils/planning_budget.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>

// Build the mission context: every POI is inserted in the roadmap graph as a node (addTemporaryNode:
// connects it to any existing graph node within `radius` that has a
// collision free straight line to it, see roadmap_utils.cpp), then Dijkstra
// is run once from each POI to get the shortest roadmap distance (and path,
// via prevs) to every other POI.
//
// ctx.poi layout matches the Orienteering matrix convention (index 0 =
// start, 1..n = victims, n+1 = gate): ctx.poi[0] is pushed first (start
// node), then victims in the same order as world.victims, then the gate.
MissionContext buildMissionContext(RoadmapGraph& roadmap, const WorldModel& world){
    MissionContext ctx;

    // Add POIs once so budget re-solves keep the sampled node count fixed.
    int startNode =
        roadmap::addTemporaryNode(roadmap, world.start.x, world.start.y, world);

    std::vector<int> victimNodes;
    for(const auto& victim : world.victims){
        victimNodes.push_back(roadmap::addTemporaryNode(roadmap, victim.x, victim.y, world));
    }

    int gateNode = roadmap::addTemporaryNode(roadmap, world.gates[0].position.x, world.gates[0].position.y, world);

    ctx.poi.push_back(startNode);
    for(int id : victimNodes){
        ctx.poi.push_back(id);
    }
    ctx.poi.push_back(gateNode);

    // ctx.D[i][j] = shortest roadmap distance from POI i to POI j.
    // ctx.prevs[i] = Dijkstra's parent-pointer array rooted at POI i, kept
    // around so solveMissionWithBudget can later call reconstructPath() to
    // recover the actual sequence of graph nodes between any two POIs (the
    // distance matrix alone only has the numbers, not the routes).
    ctx.D.assign(ctx.poi.size(), std::vector<double>(ctx.poi.size()));
    ctx.prevs.assign(ctx.poi.size(), {});

    for(size_t i=0;i<ctx.poi.size();i++){
        std::vector<double> dist;
        roadmap::dijkstra(roadmap, ctx.poi[i], dist, ctx.prevs[i]);

        for(size_t j=0;j<ctx.poi.size();j++){
            ctx.D[i][j]=dist[ctx.poi[j]];
        }
    }

    for(const auto& v : world.victims){
        ctx.values.push_back(v.value);
    }

    return ctx;
}

// Given a precomputed MissionContext (distance matrix + values) and a
// distance budget, solve the Orienteering Problem to pick which victims to
// visit and in what order. Then convert the abstract victim order into a
// concrete sequence of roadmap graph node indices the rest of the pipeline
// (tour_builder / reference_generation) can turn into (x,y) via-points.
VictimMissionResult solveMissionWithBudget(const MissionContext& ctx, double budget){
    VictimMissionResult mission;

    ROS_INFO("Budget = %.2f", budget);

    // "auto": picks exact bitmask DP or the greedy+2-opt heuristic depending
    // on victim count (see orienteering.cpp's solveOrienteering dispatcher).
    auto result = comb::solveOrienteering(ctx.D, ctx.values, budget, "auto");
    for(size_t i=0;i<ctx.D.size();i++){
        std::stringstream ss;
        for(size_t j=0;j<ctx.D[i].size();j++){
            if(std::isinf(ctx.D[i][j]))
                ss << "INF ";
            else
                ss << std::fixed
                   << std::setprecision(1)
                   << ctx.D[i][j]
                   << " ";
        }

        ROS_INFO("D[%lu] = %s",
                 i,
                 ss.str().c_str());
    }

    ROS_INFO("Selected %lu victims",
             result.victim_order.size());

    ROS_INFO("Collected value %.2f",
             result.total_value);

    // result.victim_order holds 0-based victim indices (0..n-1, matching
    // ctx.values / world.victims). Convert to POI indices in ctx.poi
    // (0=start, 1..n=victims, n+1=gate) to build the full stop sequence:
    // start -> selected victims (in the chosen order) -> gate.
    std::vector<int> poiOrder;
    poiOrder.push_back(0);
    for(int v : result.victim_order)
        poiOrder.push_back(v+1);
    poiOrder.push_back(ctx.poi.size()-1);

    // Stitch together the graph-node path: for each consecutive pair of
    // stops in poiOrder, reconstructPath() gives the exact shortest-path node
    // sequence between them (using the prevs[] Dijkstra tree rooted at the
    // first stop of that leg). Concatenate all legs into one continuous path,
    // dropping the duplicated junction node between legs (part.begin()+1)
    // since it's already the last node of the previous leg.
    std::vector<int> graphPath;
    for(size_t i=0;i+1<poiOrder.size();i++)
    {
        std::vector<int> part =
            roadmap::reconstructPath(
                ctx.poi[poiOrder[i]],
                ctx.poi[poiOrder[i+1]],
                ctx.prevs[poiOrder[i]]);

        if(part.empty())
            continue;

        if(graphPath.empty())
        {
            graphPath.insert(graphPath.end(), part.begin(), part.end());
        }
        else
        {
            graphPath.insert(graphPath.end(), part.begin()+1, part.end());
        }
    }

    mission.feasible = result.feasible;
    mission.graph_path = graphPath;
    mission.selected_victims = result.victim_order;
    mission.collected_value = result.total_value;
    mission.total_length = result.total_length;

    return mission;
}

// Top-level convenience entry point: builds the context, derives the
// distance budget from the mission's time limit (or uses an explicit
// override, e.g. for benchmarking/what-if analysis), and solves.
VictimMissionResult computeVictimMission(RoadmapGraph& roadmap, const WorldModel& world, double budgetOverride){
    MissionContext ctx = buildMissionContext(roadmap, world);

    const double v_max = 0.3;
    // Graph budget is derated: Dubins arcs exceed straight roadmap distance.
    // The roadmap distance matrix D is built from straight polyline edges,
    // but the final trajectory follows curved Dubins arcs between the same
    // waypoints, which are always >= the straight-line distance (a curve
    // can't be shorter than the straight line it connects). Multiplying the
    // time-derived budget by 0.85 leaves headroom so a plan that looks
    // "just barely feasible" in straight-line roadmap distance doesn't end
    // up violating the real timeout once Dubins curvature is added on top.
    const double dubins_safety = 0.85;
    const double budget =
        (budgetOverride >= 0.0)
            ? budgetOverride
            : comb::distanceBudget(world.victims_timeout, v_max, dubins_safety);

    return solveMissionWithBudget(ctx, budget);
}