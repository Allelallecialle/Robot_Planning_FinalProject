#pragma once

#include "task/roadmap_algorithms.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"
#include "world_model.hpp"

// `world` is needed to re-check the stitched Dubins trajectory against the
// obstacles/borders: the roadmap edges are only collision-checked as straight
// lines, but the bounded-curvature arcs the robot actually follows can bulge
// out of those segments and clip an obstacle.
std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, double start_yaw, const WorldModel& world);