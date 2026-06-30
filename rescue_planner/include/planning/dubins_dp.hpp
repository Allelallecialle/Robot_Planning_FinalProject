#pragma once

// ============================================================================
//  dubins_dp.hpp  --  optimal heading selection for a multipoint Dubins chain
// ----------------------------------------------------------------------------
//  Port of the dynamic-programming idea in
//  loco_nav/loco_planning/scripts/planners/dp.py.
//
//  Once the victim order is fixed we have a polyline of fixed WAYPOINTS
//  (start, victim centres, gate). The start and gate headings are mandated;
//  the intermediate headings (at each victim) are free. We choose them to
//  MINIMISE the total length of the stitched Dubins manoeuvres.
//
//  Method: discretize each free heading into `discretizations` candidates and
//  solve a shortest-path DP over the trellis of (waypoint, candidate-heading)
//  states, where the edge cost between consecutive waypoints is the Dubins
//  shortest-path length for the chosen pair of headings.
//      Complexity: O(P * D^2 * cost_dubins), P = #waypoints, D = #candidates.
//  We also seed each free node with the geometric "tangent" heading toward the
//  next and from the previous waypoint, which markedly improves quality at low
//  D (same trick as dp.py's guess_initial_angles).
// ============================================================================

#include <vector>
#include "planning/geometry_utils.hpp"

namespace comb {

// Returns the chosen heading (rad) at every waypoint. `pts` must have >= 2
// entries. The first heading is fixed to `start_yaw`, the last to `gate_yaw`.
std::vector<double> optimizeHeadings(const std::vector<Vec2>& pts,
                                     double start_yaw, double gate_yaw,
                                     double k_max, int discretizations = 72);

}  // namespace comb
