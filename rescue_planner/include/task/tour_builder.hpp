#pragma once

#include <string>
#include <vector>

#include "task/visibility_graph.hpp"
#include "trajectory/dubins.hpp"
#include "utils/geometry_utils.hpp"

namespace comb {

using Roadmap = VisibilityGraph;

struct TourResult {
    std::vector<RefSample> reference;
    std::vector<Vec2>      waypoints;
    std::vector<int>       victim_order;
    double total_value  = 0.0;
    double graph_length = 0.0;
    double flyable_length = 0.0;
    bool   feasible     = false;
};

TourResult planTour(const Roadmap& graph, const GeoMap& map, double start_yaw,
                    double gate_yaw, const std::vector<double>& values,
                    double Dmax, const std::string& op_method, double v_max,
                    double k_max, double dt, int dubins_discretizations,
                    double sample_res);

}  // namespace comb
