#pragma once

// ============================================================================
//  orienteering.hpp  --  prize-collecting victim selection (Orienteering Problem)
// ----------------------------------------------------------------------------
//  Given the pairwise shortest-distance matrix between {start, victims, gate}
//  (from the visibility graph + Dijkstra), choose an ORDERED subset of victims
//      start -> v_{i1} -> ... -> v_{ik} -> gate
//  that MAXIMISES the total collected value subject to the total travelled
//  distance not exceeding the budget Dmax (= v_max * timeout, derated to leave
//  headroom for the curvature overhead of the final Dubins stitching).
//
//  This is the classic Orienteering Problem (a.k.a. prize-collecting TSP).
//  Two interchangeable solvers are provided (selectable at runtime) so the
//  report can discuss the complexity/optimality trade-off:
//
//    * EXACT  -- Held-Karp style dynamic programming over victim subsets.
//        state dp[mask][i] = min distance of a walk that starts at `start`,
//        visits exactly the victim set `mask`, and ends at victim i.
//        Complexity: O(2^n * n^2) time, O(2^n * n) memory.
//        Practical for n up to ~15-18 victims (the expected size here).
//
//    * HEURISTIC -- greedy best-ratio insertion followed by 2-opt local search.
//        Complexity: roughly O(n^3) per improvement sweep; used as a fallback
//        when n is too large for the exact DP.
//
//  The matrix index convention is:  0 = start, 1..n = victims, n+1 = gate.
// ============================================================================

#include <vector>
#include <string>

namespace comb {

struct OrienteeringResult {
    std::vector<int> victim_order;  // chosen victims, 0-based victim indices
    double total_value  = 0.0;
    double total_length = 0.0;      // graph distance start->...->gate
    bool   feasible     = false;    // reaches the gate within budget
};

// `dist` is an (n+2) x (n+2) matrix with the index convention above. `values`
// has one entry per victim. `method` is "exact" or "greedy"; "auto" picks
// exact when n <= exact_limit else greedy.
OrienteeringResult solveOrienteering(const std::vector<std::vector<double>>& dist,
                                     const std::vector<double>& values,
                                     double Dmax,
                                     const std::string& method = "auto",
                                     int exact_limit = 16);

}  // namespace comb
