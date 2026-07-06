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
