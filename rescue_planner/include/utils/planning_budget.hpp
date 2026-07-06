#pragma once

// ============================================================================
//  planning_budget.hpp  --  rescue time budget (distance) for OP + trajectory
// ----------------------------------------------------------------------------
//  The course publishes a time limit on /victims_timeout [s]. The robot travels
//  at (nearly) constant v_max, so the maximum *graph* distance budget is
//
//      Dmax = v_max * victims_timeout * dubins_safety
//
//  where dubins_safety < 1 derates the budget to leave headroom for the extra
//  length of curved Dubins arcs over straight roadmap segments.
//
//  Convention (matches world_model.hpp):
//    victims_timeout <= 0  =>  unlimited budget (+infinity).
//    victims_timeout >  0  =>  finite budget as above.
// ============================================================================

#include <cmath>
#include <limits>

namespace comb {

/**
 * @brief Convert a rescue time limit into a maximum roadmap travel distance.
 *
 * @param victims_timeout_sec  Time budget in seconds (0 or negative => unlimited).
 * @param v_max                Maximum forward speed [m/s].
 * @param dubins_safety        Derating factor in (0, 1] applied to the budget.
 * @return Dmax [m]; +infinity when the timeout is unlimited.
 */
inline constexpr double distanceBudget(int victims_timeout_sec, double v_max,
                                     double dubins_safety) {
    if (victims_timeout_sec <= 0)
        return std::numeric_limits<double>::infinity();
    return v_max * static_cast<double>(victims_timeout_sec) * dubins_safety;
}

}  // namespace comb
