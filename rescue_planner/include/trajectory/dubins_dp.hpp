#pragma once

#include <vector>
#include "utils/geometry_utils.hpp"

namespace comb {

std::vector<double> optimizeHeadings(const std::vector<Vec2>& pts,
                                     double start_yaw, double gate_yaw,
                                     double k_max, int discretizations = 72);

}  // namespace comb
