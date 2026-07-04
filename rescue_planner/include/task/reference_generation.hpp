#pragma once

#include "task/roadmap_algorithms.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"

std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, double start_yaw);