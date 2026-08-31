#pragma once

#include "world_model.hpp"
#include "task/roadmap_algorithms.hpp"

#include <vector>

struct VictimMissionResult
{
    bool feasible = false;
    std::vector<int> graph_path;
    std::vector<int> selected_victims;
    double collected_value = 0.0;
    double total_length = 0.0;

    // Roadmap node id of every visited POI (start, selected victims in visiting
    // order, gate), i.e. the leg boundaries within graph_path. Needed downstream
    // so per-leg line-of-sight simplification never shortcuts across a required
    // victim stop -- see generateReferenceFromGraphPath.
    std::vector<int> poi_path_nodes;
};

struct MissionContext
{
    std::vector<int> poi;
    std::vector<std::vector<double>> D;
    std::vector<std::vector<int>> prevs;
    std::vector<double> values;
};

MissionContext buildMissionContext(RoadmapGraph& roadmap, const WorldModel& world);
VictimMissionResult solveMissionWithBudget(const MissionContext& ctx, double budget);
VictimMissionResult computeVictimMission(RoadmapGraph& roadmap, const WorldModel& world, double budgetOverride = -1.0);
