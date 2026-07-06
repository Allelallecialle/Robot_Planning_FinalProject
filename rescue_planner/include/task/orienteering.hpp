#pragma once

#include <vector>
#include <string>

namespace comb {

struct OrienteeringResult {
    std::vector<int> victim_order;
    double total_value  = 0.0;
    double total_length = 0.0;
    bool   feasible     = false;
};

// Matrix index convention: 0 = start, 1..n = victims, n+1 = gate.
OrienteeringResult solveOrienteering(const std::vector<std::vector<double>>& dist,
                                     const std::vector<double>& values,
                                     double Dmax,
                                     const std::string& method = "auto",
                                     int exact_limit = 16);

}  // namespace comb
