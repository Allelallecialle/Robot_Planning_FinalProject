#pragma once

#include "task/roadmap_algorithms.hpp"
#include "task/victim_mission.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"
#include "world_model.hpp"

// poi_path_nodes marks the leg boundaries within graph_path (see
// VictimMissionResult::poi_path_nodes): line-of-sight simplification is run
// independently on each POI-to-POI leg so a required victim stop can never be
// shortcut past, mirroring the combinatorial planners' task/tour_builder.cpp.
std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, const std::vector<int>& poi_path_nodes, double start_yaw, const WorldModel& world);

struct SamplingMissionPlan {
    VictimMissionResult mission;
    std::vector<comb::RefSample> reference;
};

SamplingMissionPlan planSamplingMission(RoadmapGraph& roadmap, const WorldModel& world, double start_yaw);
