#include "task/victim_mission.hpp"

#include "task/orienteering.hpp"
#include "utils/roadmap_utils.hpp"
#include "utils/planning_budget.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>

VictimMissionResult computeVictimMission(RoadmapGraph& roadmap, const WorldModel& world, double budgetOverride){
    VictimMissionResult mission;
    int startNode =
    roadmap::addTemporaryNode(roadmap, world.start.x, world.start.y, world);

    std::vector<int> victimNodes;

    for(const auto& victim : world.victims){
        victimNodes.push_back(roadmap::addTemporaryNode(roadmap, victim.x, victim.y, world));
    }

    int gateNode = roadmap::addTemporaryNode(roadmap, world.gates[0].position.x, world.gates[0].position.y, world);

    std::vector<int> poi;
    poi.push_back(startNode);

    for(int id : victimNodes){
        poi.push_back(id);
    }

    poi.push_back(gateNode);

    std::vector<std::vector<double>> D(poi.size(), std::vector<double>(poi.size()));
    std::vector<std::vector<int>> prevs(poi.size());

    for(size_t i=0;i<poi.size();i++){
        std::vector<double> dist;
        roadmap::dijkstra(roadmap, poi[i], dist, prevs[i]);

        for(size_t j=0;j<poi.size();j++){
            D[i][j]=dist[poi[j]];
        }
    }

    std::vector<double> values;
    for(const auto& v : world.victims){
        values.push_back(v.value);
    }
    const double v_max = 0.3;
    // Derating factor in (0, 1]: the flyable Dubins trajectory is always LONGER
    // than the straight roadmap path, so the graph-distance budget must be
    // SMALLER than v_max * timeout. Matches the combinatorial planners (0.85).
    // (A value > 1 would over-estimate the budget and the robot would miss the
    // timeout.)
    const double dubins_safety = 0.85;
    // Use the caller-supplied budget when the flyable-time feedback loop is
    // tightening it; otherwise fall back to the default timeout-derived budget.
    const double budget =
        (budgetOverride >= 0.0)
            ? budgetOverride
            : comb::distanceBudget(world.victims_timeout, v_max, dubins_safety);

    ROS_INFO("Victims timeout = %.2f",
             world.victims_timeout);

    ROS_INFO("Budget = %.2f",
             budget);

    auto result = comb::solveOrienteering(D, values, budget, "auto");
    for(size_t i=0;i<D.size();i++){
        std::stringstream ss;
        for(size_t j=0;j<D[i].size();j++){
            if(std::isinf(D[i][j]))
                ss << "INF ";
            else
                ss << std::fixed
                   << std::setprecision(1)
                   << D[i][j]
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

    poiOrder.push_back(poi.size()-1);

    std::vector<int> graphPath;

    for(size_t i=0;i+1<poiOrder.size();i++)
    {
        std::vector<int> part =
            roadmap::reconstructPath(
                poi[poiOrder[i]],
                poi[poiOrder[i+1]],
                prevs[poiOrder[i]]);

        if(part.empty())
            continue;

        if(graphPath.empty())
        {
            graphPath.insert(
                graphPath.end(),
                part.begin(),
                part.end());
        }
        else
        {
            graphPath.insert(
                graphPath.end(),
                part.begin()+1,
                part.end());
        }
    }

    mission.feasible =
    result.feasible;

    mission.graph_path =
        graphPath;

    mission.selected_victims =
        result.victim_order;

    mission.collected_value =
        result.total_value;

    mission.total_length =
        result.total_length;

    return mission;
}