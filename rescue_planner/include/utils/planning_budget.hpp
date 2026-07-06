#pragma once

#include <cmath>
#include <limits>

namespace comb {

inline constexpr double distanceBudget(int victims_timeout_sec, double v_max,
                                     double dubins_safety) {
    if (victims_timeout_sec <= 0)
        return std::numeric_limits<double>::infinity();
    return v_max * static_cast<double>(victims_timeout_sec) * dubins_safety;
}

}  // namespace comb
