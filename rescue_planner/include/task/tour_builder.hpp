#pragma once

// ============================================================================
//  tour_builder.hpp  --  shared "roadmap -> tour -> flyable path" pipeline
// ----------------------------------------------------------------------------
//  The three combinatorial planners of this package (visibility graph, exact
//  cell decomposition and Voronoi/GVD roadmap) share the very same high-level
//  pipeline; ONLY the roadmap-construction stage differs. This module captures
//  the common downstream stages so they are implemented exactly once and can be
//  unit-tested WITHOUT a running ROS master:
//
//     roadmap (comb::VisibilityGraph, POI indices filled)
//        -> all-pairs Dijkstra between the points of interest      (collapse)
//        -> Orienteering (pick victim subset/order within budget)  (selection)
//        -> heading DP + Dubins stitching with clearance re-check   (path)
//
//  The heading/stitching stage mirrors visibility_planner.cpp exactly: after
//  stitching, each curved arc is re-sampled and verified against the inflated
//  obstacles; if a leg clips, the midpoint of the offending straight segment
//  (guaranteed clear, since it came from the roadmap) is inserted and the DP is
//  re-run, which shrinks the arc.
//
//  The container type is `comb::VisibilityGraph` (aliased below as
//  `comb::Roadmap`): it is a plain nodes + adjacency list with the POI indices
//  attached, which is exactly what every combinatorial roadmap needs. Dijkstra
//  derives the Euclidean edge weight from the node coordinates, so callers only
//  have to record adjacency (never weights).
// ============================================================================

#include <string>
#include <vector>

#include "task/visibility_graph.hpp"
#include "trajectory/dubins.hpp"
#include "utils/geometry_utils.hpp"

namespace comb {

// The generic combinatorial roadmap container. The visibility graph struct
// already carries {nodes, adjacency, start/victim/gate indices}, i.e. exactly
// what the cell-decomposition and Voronoi roadmaps also need, so we reuse it
// rather than duplicating the type (and its Dijkstra) three times.
using Roadmap = VisibilityGraph;

/**
 * @brief Result of the shared tour-planning pipeline.
 *
 * All geometry is expressed in world-space metres. `reference` is the final
 * discretized, flyable Dubins trajectory (constant speed v_max); it is empty
 * when the problem is infeasible.
 */
struct TourResult {
    std::vector<RefSample> reference;   ///< discretized flyable path (may be empty)
    std::vector<Vec2>      waypoints;   ///< POI-expanded polyline (roadmap nodes)
    std::vector<int>       victim_order;///< chosen victims, 0-based victim indices
    double total_value  = 0.0;          ///< collected reward
    double graph_length = 0.0;          ///< straight-line tour length on the roadmap
    double flyable_length = 0.0;        ///< length of the stitched Dubins path
    bool   feasible     = false;        ///< reaches the gate within the budget
};

/**
 * @brief Collapse a roadmap into a flyable rescue tour (OP + Dubins layer).
 *
 * Runs Dijkstra from every point of interest to obtain the pairwise distance
 * matrix, solves the Orienteering Problem to select and order the victims
 * within `Dmax`, expands the chosen POI order back into a geometric polyline
 * and stitches bounded-curvature Dubins manoeuvres between the waypoints, with
 * an iterative midpoint-subdivision repair loop that guarantees the final
 * curved path is collision-free in the inflated map.
 *
 * @param graph        Roadmap with `start_idx`, `victim_idx`, `gate_idx` set.
 * @param map          Inflated geometry, used for the curved-arc clearance recheck.
 * @param start_yaw    Mandatory heading at the start pose (rad).
 * @param gate_yaw     Mandatory heading at the gate pose (rad).
 * @param values       Reward of each victim (same order as `graph.victim_idx`).
 * @param Dmax         Distance budget (+inf when unlimited). Non-positive => infeasible.
 * @param op_method    "auto" | "exact" | "greedy" orienteering solver selector.
 * @param v_max        Constant forward speed used to time-parametrise the path.
 * @param k_max        Maximum curvature (1 / turning_radius).
 * @param dt           Discretization time step of the reference trajectory.
 * @param dubins_discretizations  Number of candidate headings per free waypoint.
 * @param sample_res   Collision sampling resolution used elsewhere (unused here
 *                     directly but kept for signature symmetry / future use).
 * @return TourResult; `feasible == false` (and empty reference) if no tour fits.
 * @complexity O(P * E log V) for the P Dijkstra runs plus the orienteering and
 *             the O(subdiv * P * D^2) heading DP; V,E are the roadmap sizes.
 */
TourResult planTour(const Roadmap& graph, const GeoMap& map, double start_yaw,
                    double gate_yaw, const std::vector<double>& values,
                    double Dmax, const std::string& op_method, double v_max,
                    double k_max, double dt, int dubins_discretizations,
                    double sample_res);

}  // namespace comb
