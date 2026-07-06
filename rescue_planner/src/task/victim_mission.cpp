#include "task/victim_mission.hpp"

#include "task/orienteering.hpp"
#include "utils/roadmap_utils.hpp"
#include "utils/planning_budget.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>

MissionContext buildMissionContext(RoadmapGraph& roadmap, const WorldModel& world){
    MissionContext ctx;

    // Add the POIs to the roadmap ONCE. Doing this here (rather than inside a
    // budget re-solve) is what keeps the roadmap node count fixed across the
    // flyable-time feedback loop -- the sampled node count set by the planner
    // (e.g. 1500 for RRT, 500 for PRM) is not silently grown by re-planning.
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

VictimMissionResult solveMissionWithBudget(const MissionContext& ctx, double budget){
    VictimMissionResult mission;

    ROS_INFO("Budget = %.2f", budget);

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

    std::vector<int> poiOrder;
    poiOrder.push_back(0);
    for(int v : result.victim_order)
        poiOrder.push_back(v+1);
    poiOrder.push_back(ctx.poi.size()-1);

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

VictimMissionResult computeVictimMission(RoadmapGraph& roadmap, const WorldModel& world, double budgetOverride){
    MissionContext ctx = buildMissionContext(roadmap, world);

    const double v_max = 0.3;
    // Derating factor in (0, 1]: the flyable Dubins trajectory is always LONGER
    // than the straight roadmap path, so the graph-distance budget must be
    // SMALLER than v_max * timeout. Matches the combinatorial planners (0.85).
    const double dubins_safety = 0.85;
    const double budget =
        (budgetOverride >= 0.0)
            ? budgetOverride
            : comb::distanceBudget(world.victims_timeout, v_max, dubins_safety);

    return solveMissionWithBudget(ctx, budget);
}