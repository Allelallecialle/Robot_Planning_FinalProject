#pragma once

#include <vector>

#include "utils/geometry_utils.hpp"
#include "trajectory/dubins_dp.hpp"

namespace reference_builder
{

std::vector<comb::RefSample> buildReference(
    const std::vector<comb::Vec2>& waypoints,
    double startYaw,
    double goalYaw,
    double vmax,
    double dt,
    double kmax,
    int headingDiscretizations);

}