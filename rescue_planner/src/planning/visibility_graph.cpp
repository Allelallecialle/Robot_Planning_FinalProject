#include "planning/visibility_graph.hpp"

#include <algorithm>
#include <limits>
#include <queue>

namespace comb {

int VisibilityGraph::addNode(const Vec2& p) {
    nodes.push_back(p);
    adj.emplace_back();
    return static_cast<int>(nodes.size()) - 1;
}

VisibilityGraph buildVisibilityGraph(const GeoMap& map, const Vec2& start,
                                     const std::vector<Vec2>& victims,
                                     const Vec2& gate, double node_buffer,
                                     double sample_res) {
    VisibilityGraph g;

    // --- POIs always become nodes (even if they sit close to clearance) ---
    g.start_idx = g.addNode(start);
    g.victim_idx.reserve(victims.size());
    for (const auto& v : victims) g.victim_idx.push_back(g.addNode(v));
    g.gate_idx = g.addNode(gate);

    // --- obstacle corner waypoints (inflated polygon vertices) ---
    // node_buffer is slightly larger than map.clearance so that chords between
    // adjacent corners of the same obstacle remain in free space.
    for (const auto& obs : map.obstacles) {
        if (obs.is_circle) continue;  // circles have no corners; POIs route around
        const std::vector<Vec2> infl =
            inflatedPolygonVertices(obs.verts, node_buffer, map);
        for (const auto& q : infl) g.addNode(q);
    }

    // --- edges: connect every pair whose connecting segment is clear ---
    const int n = static_cast<int>(g.nodes.size());
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (segmentClear(g.nodes[i], g.nodes[j], map, sample_res)) {
                g.adj[i].push_back(j);
                g.adj[j].push_back(i);
            }
        }
    }
    return g;
}

void dijkstra(const VisibilityGraph& g, int src, std::vector<double>& dist,
              std::vector<int>& prev) {
    const int n = static_cast<int>(g.nodes.size());
    dist.assign(n, std::numeric_limits<double>::infinity());
    prev.assign(n, -1);
    if (src < 0 || src >= n) return;

    using Item = std::pair<double, int>;  // (distance, node)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        const auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;  // stale entry
        for (int v : g.adj[u]) {
            const double w = dist2(g.nodes[u], g.nodes[v]);
            const double nd = d + std::sqrt(w);
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }
}

std::vector<int> reconstructPath(int src, int dst,
                                 const std::vector<int>& prev) {
    std::vector<int> path;
    if (dst < 0 || dst >= static_cast<int>(prev.size())) return path;
    int cur = dst;
    while (cur != -1) {
        path.push_back(cur);
        if (cur == src) break;
        cur = prev[cur];
    }
    if (path.empty() || path.back() != src) return {};  // unreachable
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace comb
