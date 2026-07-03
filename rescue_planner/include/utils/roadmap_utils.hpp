#pragma once

#include "task/roadmap_graph.hpp"
#include "world_model.hpp"

namespace roadmap
{
int addTemporaryNode(RoadmapGraph& graph, double x, double y, const WorldModel& world, double radius = 2.0);
}