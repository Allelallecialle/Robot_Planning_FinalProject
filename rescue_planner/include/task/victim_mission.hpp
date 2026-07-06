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

// Prebuilt, budget-independent data for the orienteering solve. Building this
// ADDS the start/victim/gate POIs to the roadmap ONCE (via addTemporaryNode) and
// runs the all-pairs Dijkstra, so it must be constructed a single time. It can
// then be reused for any number of budget re-solves without touching the roadmap
// again -- this is what keeps the sampling roadmap's node count fixed while the
// flyable-time feedback loop tightens the budget (see planSamplingMission).
struct MissionContext
{
    std::vector<int> poi;                        // roadmap node id per POI
    std::vector<std::vector<double>> D;          // POI-POI shortest distances
    std::vector<std::vector<int>> prevs;         // Dijkstra predecessors per POI
    std::vector<double> values;                  // victim values (POI order)
};

// Adds the POIs to `roadmap` ONCE and precomputes the POI-POI distance matrix.
// Call exactly once per roadmap, then reuse the result for budget re-solves.
MissionContext buildMissionContext(RoadmapGraph& roadmap, const WorldModel& world);

// Solves the orienteering problem on a PREBUILT context for the given budget and
// reconstructs the graph path. Does NOT mutate the roadmap, so it is safe to
// call repeatedly with different budgets.
VictimMissionResult solveMissionWithBudget(const MissionContext& ctx, double budget);

// `budgetOverride` lets the caller impose an explicit graph-distance budget
// instead of the default one derived from world.victims_timeout. A negative
// value keeps the default (timeout-derived) budget. This is the convenience
// one-shot entry point (build context + solve); the feedback loop instead uses
// buildMissionContext + solveMissionWithBudget so the roadmap is not regrown.
VictimMissionResult computeVictimMission(RoadmapGraph& roadmap, const WorldModel& world, double budgetOverride = -1.0);