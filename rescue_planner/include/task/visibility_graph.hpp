#pragma once

#include <vector>
#include "utils/geometry_utils.hpp"

namespace comb {

struct VisibilityGraph {
    std::vector<Vec2>              nodes;
    std::vector<std::vector<int>> adj;

    int start_idx = -1;
    int gate_idx  = -1;
    std::vector<int> victim_idx;

    int addNode(const Vec2& p);
};

VisibilityGraph buildVisibilityGraph(const GeoMap& map, const Vec2& start,
                                     const std::vector<Vec2>& victims,
                                     const Vec2& gate, double node_buffer,
                                     double sample_res);
void dijkstra(const VisibilityGraph& g, int src, std::vector<double>& dist,
              std::vector<int>& prev);
std::vector<int> reconstructPath(int src, int dst, const std::vector<int>& prev);

}  // namespace comb
