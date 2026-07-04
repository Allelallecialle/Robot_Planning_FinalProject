#pragma once

#include <vector>
#include <loco_planning/Reference.h>
#include <ros/ros.h>
#include <thread>


#include "sampler.hpp"
#include "collision_checker.hpp"
#include "task/roadmap_algorithms.hpp"
#include "task/orienteering.hpp"
#include "utils/roadmap_utils.hpp"
#include "utils/geometry_utils.hpp"
#include "task/victim_mission.hpp"
#include "task/reference_generation.hpp"

void publishRef(const std::vector<comb::RefSample>& ref, ros::Publisher pub);