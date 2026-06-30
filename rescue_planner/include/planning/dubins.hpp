#pragma once

// ============================================================================
//  dubins.hpp  --  Dubins shortest-path primitives (C++ port of dubins.py)
// ----------------------------------------------------------------------------
//  Faithful port of loco_nav/loco_planning/scripts/planners/dubins.py so the
//  combinatorial planner produces exactly the same flyable manoeuvres the
//  sampling-based planners rely on (constant forward speed, bounded curvature
//  Kmax = 1/turning_radius). A Dubins curve is the concatenation of three arcs
//  (each straight or a circular arc of curvature +-Kmax). We try all six
//  primitives (LSL, RSR, LSR, RSL, RLR, LRL) and keep the shortest feasible.
// ============================================================================

#include <vector>
#include <array>
#include <cmath>

namespace comb {

struct DubinsArc {
    double x0, y0, th0;  // arc start pose
    double k;            // curvature (signed)
    double L;            // arc length
    double xf, yf, thf;  // arc end pose
};

struct DubinsCurve {
    bool   valid = false;
    DubinsArc a1, a2, a3;
    double L = 0.0;      // total length
};

// One discretized reference sample along a (multi-)Dubins trajectory.
struct RefSample {
    double x, y, theta, v, omega, t;
};

// Compute the shortest Dubins path between two oriented poses. `Kmax` is the
// maximum curvature (1/turning_radius). Returns curve.valid == false if no
// feasible primitive exists (e.g. coincident points).
DubinsCurve dubinsShortestPath(double x0, double y0, double th0,
                               double xf, double yf, double thf, double Kmax);

// Discretize a Dubins curve into reference samples at fixed time step dt using
// the constant linear velocity v_max (the scenario forbids slowing/stopping).
// `t_offset` is added to every sample's timestamp so multi-leg trajectories
// keep a monotonically increasing time. Samples are appended to `out`.
void appendDiscretizedDubins(const DubinsCurve& curve, double v_max, double dt,
                             double t_offset, std::vector<RefSample>& out);

// Evaluate a point along an arc of given curvature (used for clearance checks).
void circline(double s, double x0, double y0, double th0, double k,
              double& x, double& y, double& th);

double mod2pi(double theta);

}  // namespace comb
