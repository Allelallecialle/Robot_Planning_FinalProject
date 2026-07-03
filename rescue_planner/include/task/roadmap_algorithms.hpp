#pragma once

#include <vector>
#include "task/roadmap_graph.hpp"

namespace roadmap{

void dijkstra(const RoadmapGraph& graph, int source, std::vector<double>& dist, std::vector<int>& prev);

std::vector<int> reconstructPath(int source, int target, const std::vector<int>& prev);

std::vector<std::vector<double>>
computeDistanceMatrix(const RoadmapGraph& graph, const std::vector<int>& poi_nodes);

}