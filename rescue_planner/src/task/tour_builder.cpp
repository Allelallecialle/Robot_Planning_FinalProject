#include "task/tour_builder.hpp"

#include <cstddef>
#include <limits>

#include "task/orienteering.hpp"
#include "trajectory/dubins_dp.hpp"

namespace comb {

namespace {

// Greedy line-of-sight simplification of a roadmap sub-path, preserving the two
// endpoints (which are the points of interest that MUST be visited). From the
// current vertex we jump to the farthest later vertex still reachable by a
// clearance-safe straight segment. This is essential for the dense grid-based
// roadmaps (Voronoi): stitching bounded-curvature Dubins arcs between hundreds
// of ~cell-spaced waypoints would explode the path length; every retained
// shortcut still keeps clearance >= r_c (segmentClear), so the pruned polyline
// remains collision-safe.
std::vector<Vec2> simplifyLineOfSight(const std::vector<Vec2>& poly,
                                      const GeoMap& map, double sample_res) {
    if (poly.size() <= 2) return poly;
    std::vector<Vec2> out;
    out.push_back(poly.front());
    std::size_t i = 0;
    const std::size_t n = poly.size();
    while (i < n - 1) {
        std::size_t j = n - 1;
        while (j > i + 1 && !segmentClear(poly[i], poly[j], map, sample_res))
            --j;
        out.push_back(poly[j]);
        i = j;
    }
    return out;
}

}  // namespace

TourResult planTour(const Roadmap& graph, const GeoMap& map, double start_yaw,
                    double gate_yaw, const std::vector<double>& values,
                    double Dmax, const std::string& op_method, double v_max,
                    double k_max, double dt, int dubins_discretizations,
                    double sample_res) {
    TourResult out;
    if (graph.start_idx < 0 || graph.gate_idx < 0) return out;

    // ---------- 1) all-pairs shortest distances between POIs ------------------
    // POI index convention (matching orienteering.hpp): 0 = start, 1..n =
    // victims, n+1 = gate.
    std::vector<int> poi_node;
    poi_node.push_back(graph.start_idx);
    for (int idx : graph.victim_idx) poi_node.push_back(idx);
    poi_node.push_back(graph.gate_idx);
    const int P = static_cast<int>(poi_node.size());  // == n + 2

    std::vector<std::vector<double>> D(P, std::vector<double>(P, 0.0));
    std::vector<std::vector<int>> prevs(P);  // Dijkstra predecessor per POI
    for (int p = 0; p < P; ++p) {
        std::vector<double> dist;
        dijkstra(graph, poi_node[p], dist, prevs[p]);
        for (int q = 0; q < P; ++q) D[p][q] = dist[poi_node[q]];
    }

    // ---------- 2) orienteering: pick victim subset & order -------------------
    const OrienteeringResult op =
        solveOrienteering(D, values, Dmax, op_method);
    out.total_value = op.total_value;
    out.graph_length = op.total_length;
    out.victim_order = op.victim_order;
    out.feasible = op.feasible;
    if (!op.feasible) return out;

    // ---------- 3) expand POI order into a geometric waypoint polyline --------
    // Each POI-to-POI leg is reconstructed on the roadmap, then simplified by
    // clearance-safe line-of-sight shortcutting (see simplifyLineOfSight). The
    // POI endpoints (start, victims, gate) are always preserved, so every
    // selected victim is still visited.
    std::vector<int> poi_order;
    poi_order.push_back(0);                       // start
    for (int vi : op.victim_order) poi_order.push_back(vi + 1);
    poi_order.push_back(P - 1);                   // gate

    std::vector<Vec2> waypoints;
    for (std::size_t k = 0; k + 1 < poi_order.size(); ++k) {
        const int a = poi_order[k];
        const int b = poi_order[k + 1];
        const std::vector<int> seg =
            reconstructPath(poi_node[a], poi_node[b], prevs[a]);
        if (seg.empty()) continue;  // POI unreachable on the roadmap

        std::vector<Vec2> segpts;
        segpts.reserve(seg.size());
        for (int idx : seg) segpts.push_back(graph.nodes[idx]);
        const std::vector<Vec2> simp =
            simplifyLineOfSight(segpts, map, sample_res);

        for (std::size_t t = (waypoints.empty() ? 0 : 1); t < simp.size(); ++t) {
            if (waypoints.empty() || dist(waypoints.back(), simp[t]) > 1e-6)
                waypoints.push_back(simp[t]);
        }
    }
    if (waypoints.size() < 2) {
        out.feasible = false;
        return out;
    }
    out.waypoints = waypoints;

    // ---------- 4) heading DP + Dubins stitching with clearance re-check ------
    // Start/gate headings are fixed; the intermediate (victim/corner) headings
    // are chosen by the DP. Each stitched arc is re-sampled against the inflated
    // map: if a leg clips, we insert the midpoint of the offending straight leg
    // (clear by construction) and re-optimise, which shrinks the bulging arc.
    std::vector<Vec2> pts = waypoints;
    std::vector<RefSample> ref;
    const int max_subdiv = 12;
    for (int iter = 0; iter <= max_subdiv; ++iter) {
        const std::vector<double> ang = optimizeHeadings(
            pts, start_yaw, gate_yaw, k_max, dubins_discretizations);

        ref.clear();
        double t_off = 0.0;
        int bad_leg = -1;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            const DubinsCurve c = dubinsShortestPath(
                pts[i].x, pts[i].y, ang[i], pts[i + 1].x, pts[i + 1].y,
                ang[i + 1], k_max);
            if (!c.valid) { bad_leg = static_cast<int>(i); break; }

            std::vector<RefSample> leg;
            appendDiscretizedDubins(c, v_max, dt, t_off, leg);

            bool clear = true;
            for (const auto& s : leg) {
                if (pointInCollision({s.x, s.y}, map)) { clear = false; break; }
            }
            if (!clear) { bad_leg = static_cast<int>(i); break; }

            for (std::size_t s = (i == 0 ? 0 : 1); s < leg.size(); ++s)
                ref.push_back(leg[s]);
            t_off = ref.empty() ? 0.0 : ref.back().t + dt;
        }

        if (bad_leg < 0) break;  // fully clear
        const Vec2 mid{(pts[bad_leg].x + pts[bad_leg + 1].x) / 2.0,
                       (pts[bad_leg].y + pts[bad_leg + 1].y) / 2.0};
        pts.insert(pts.begin() + bad_leg + 1, mid);
    }

    out.reference = ref;
    out.flyable_length = v_max * (ref.empty() ? 0.0 : ref.back().t);
    out.feasible = !ref.empty();
    return out;
}

}  // namespace comb
