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

VictimMissionResult computeVictimMission(RoadmapGraph& roadmap, const WorldModel& world);