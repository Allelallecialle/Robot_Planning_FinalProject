#pragma once

#include "task/roadmap_algorithms.hpp"
#include "task/victim_mission.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"
#include "world_model.hpp"

// `world` is needed to re-check the stitched Dubins trajectory against the
// obstacles/borders: the roadmap edges are only collision-checked as straight
// lines, but the bounded-curvature arcs the robot actually follows can bulge
// out of those segments and clip an obstacle.
//
// Returns an EMPTY reference if a leg still clips an obstacle after all the
// midpoint subdivisions: the partial (truncated) trajectory would drive the
// robot toward the obstacle and never reach the gate, so it must be rejected
// rather than published. (Same guard as comb::planTour.)
std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, double start_yaw, const WorldModel& world);

// Result of the sampling-planner mission planner: the selected victim tour on
// the roadmap plus the flyable Dubins reference the robot will execute.
struct SamplingMissionPlan {
    VictimMissionResult mission;
    std::vector<comb::RefSample> reference;
};

// Plans a rescue tour on a sampling-based roadmap AND enforces the timeout on
// the FLYABLE trajectory. computeVictimMission selects victims on the straight
// graph-distance budget, but the robot flies longer bounded-curvature arcs, so
// the stitched trajectory can overrun the timeout even when the graph tour fits.
// This picks victims, stitches the arcs, and while the flyable duration exceeds
// world.victims_timeout it tightens the graph budget in proportion and re-plans
// (dropping the victims that do not actually fit). Mirrors the feedback loop in
// the combinatorial planners so all planners respect the timeout consistently.
SamplingMissionPlan planSamplingMission(RoadmapGraph& roadmap, const WorldModel& world, double start_yaw);