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

// `budgetOverride` lets the caller impose an explicit graph-distance budget
// instead of the default one derived from world.victims_timeout. It is used by
// the flyable-time feedback loop (see planSamplingMission): the first pass runs
// with the default budget, then the budget is tightened and this is re-run so
// the FLYABLE Dubins trajectory (not just the straight graph path) fits the
// timeout. A negative value keeps the default (timeout-derived) budget.
VictimMissionResult computeVictimMission(RoadmapGraph& roadmap, const WorldModel& world, double budgetOverride = -1.0);