#pragma once

#include "task/roadmap_algorithms.hpp"
#include "task/victim_mission.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"
#include "world_model.hpp"

std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, double start_yaw, const WorldModel& world);

struct SamplingMissionPlan {
    VictimMissionResult mission;
    std::vector<comb::RefSample> reference;
};

SamplingMissionPlan planSamplingMission(RoadmapGraph& roadmap, const WorldModel& world, double start_yaw);
