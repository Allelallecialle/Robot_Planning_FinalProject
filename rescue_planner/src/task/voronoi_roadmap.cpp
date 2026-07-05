#include "task/voronoi_roadmap.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>

namespace comb {

namespace {

// Nearest-obstacle identity of a point: 0 == outside the border (wall), k+1 ==
// inside obstacle k, -1 == FREE (inside the border and outside every obstacle).
// Occupancy uses the RAW obstacles so the distance transform equals the true
// clearance to the obstacle boundary (see the header for the rationale).
int occupancyId(const Vec2& p, const GeoMap& map) {
    if (!map.border.empty() && !pointInPolygon(p, map.border)) return 0;
    for (std::size_t k = 0; k < map.obstacles.size(); ++k) {
        const Obstacle& o = map.obstacles[k];
        if (o.is_circle) {
            if (dist(p, o.center) <= o.radius) return static_cast<int>(k) + 1;
        } else {
            if (pointInPolygon(p, o.verts)) return static_cast<int>(k) + 1;
        }
    }
    return -1;
}

}  // namespace

VoronoiGrid buildDistanceTransform(const GeoMap& map, double res) {
    VoronoiGrid g;
    g.res = (res > 1e-6) ? res : 0.05;
    if (map.border.empty()) return g;

    double min_x = map.border.front().x, max_x = min_x;
    double min_y = map.border.front().y, max_y = min_y;
    for (const auto& p : map.border) {
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }

    // Pad the grid so exterior cells represent the border wall as an obstacle.
    const int pad = 3;
    g.origin_x = min_x - pad * g.res;
    g.origin_y = min_y - pad * g.res;
    g.nx = static_cast<int>(std::ceil((max_x - min_x) / g.res)) + 2 * pad + 1;
    g.ny = static_cast<int>(std::ceil((max_y - min_y) / g.res)) + 2 * pad + 1;
    const int N = g.nx * g.ny;

    g.occupied.assign(N, 0);
    g.dist.assign(N, std::numeric_limits<double>::infinity());
    g.obstacle_id.assign(N, -1);
    std::vector<int> feature(N, -1);  // linear index of nearest occupied cell

    using Item = std::pair<double, int>;  // (distance, cell)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;

    for (int j = 0; j < g.ny; ++j) {
        for (int i = 0; i < g.nx; ++i) {
            const int id = occupancyId(g.world(i, j), map);
            if (id >= 0) {
                const int c = g.index(i, j);
                g.occupied[c] = 1;
                g.dist[c] = 0.0;
                g.obstacle_id[c] = id;
                feature[c] = c;
                pq.push({0.0, c});
            }
        }
    }

    // Feature-propagation Dijkstra (8-connectivity): each FREE cell inherits the
    // nearest occupied seed, recomputing the exact Euclidean distance to it.
    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!pq.empty()) {
        const auto [d, c] = pq.top();
        pq.pop();
        if (d > g.dist[c]) continue;
        const int ci = c % g.nx;
        const int cj = c / g.nx;
        const int f = feature[c];
        const int fi = f % g.nx;
        const int fj = f / g.nx;
        for (int k = 0; k < 8; ++k) {
            const int ni = ci + di[k];
            const int nj = cj + dj[k];
            if (!g.inBounds(ni, nj)) continue;
            const int nc = g.index(ni, nj);
            const double nd =
                g.res * std::hypot(static_cast<double>(ni - fi),
                                   static_cast<double>(nj - fj));
            if (nd < g.dist[nc]) {
                g.dist[nc] = nd;
                feature[nc] = f;
                g.obstacle_id[nc] = g.obstacle_id[c];
                pq.push({nd, nc});
            }
        }
    }
    return g;
}

std::vector<int> extractSkeleton(const VoronoiGrid& grid, double clearance,
                                 int thin_iters) {
    const int N = grid.nx * grid.ny;
    std::vector<std::uint8_t> mask(N, 0);

    // Ridge candidates: FREE, clearance >= r_c, and equidistant from two
    // distinct obstacles (a differing nearest-obstacle id among 8 neighbours).
    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int j = 0; j < grid.ny; ++j) {
        for (int i = 0; i < grid.nx; ++i) {
            const int c = grid.index(i, j);
            if (grid.occupied[c]) continue;
            if (grid.dist[c] < clearance) continue;
            bool ridge = false;
            for (int k = 0; k < 8; ++k) {
                const int ni = i + di[k], nj = j + dj[k];
                if (!grid.inBounds(ni, nj)) continue;
                if (grid.obstacle_id[grid.index(ni, nj)] != grid.obstacle_id[c]) {
                    ridge = true;
                    break;
                }
            }
            if (ridge) mask[c] = 1;
        }
    }

    // Zhang-Suen thinning (preserves connectivity and endpoints). Neighbours are
    // labelled p2..p9 clockwise starting north; out-of-bounds counts as 0.
    auto P = [&](int i, int j) -> int {
        return grid.inBounds(i, j) && mask[grid.index(i, j)] ? 1 : 0;
    };
    bool changed = true;
    int iter = 0;
    while (changed && iter < thin_iters) {
        changed = false;
        ++iter;
        for (int step = 0; step < 2; ++step) {
            std::vector<int> to_delete;
            for (int j = 0; j < grid.ny; ++j) {
                for (int i = 0; i < grid.nx; ++i) {
                    if (!mask[grid.index(i, j)]) continue;
                    const int p2 = P(i, j + 1), p3 = P(i + 1, j + 1),
                              p4 = P(i + 1, j), p5 = P(i + 1, j - 1),
                              p6 = P(i, j - 1), p7 = P(i - 1, j - 1),
                              p8 = P(i - 1, j), p9 = P(i - 1, j + 1);
                    const int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                    if (B < 2 || B > 6) continue;
                    const int seq[9] = {p2, p3, p4, p5, p6, p7, p8, p9, p2};
                    int A = 0;
                    for (int t = 0; t < 8; ++t)
                        if (seq[t] == 0 && seq[t + 1] == 1) ++A;
                    if (A != 1) continue;
                    if (step == 0) {
                        if (p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                            to_delete.push_back(grid.index(i, j));
                    } else {
                        if (p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
                            to_delete.push_back(grid.index(i, j));
                    }
                }
            }
            for (int c : to_delete) mask[c] = 0;
            if (!to_delete.empty()) changed = true;
        }
    }

    std::vector<int> skeleton;
    for (int c = 0; c < N; ++c)
        if (mask[c]) skeleton.push_back(c);
    return skeleton;
}

VoronoiRoadmap buildVoronoiGraph(const VoronoiGrid& grid,
                                 const std::vector<int>& skeleton,
                                 const GeoMap& map, const Vec2& start,
                                 const std::vector<Vec2>& victims,
                                 const Vec2& gate, double sample_res) {
    VoronoiRoadmap out;
    out.grid = grid;
    out.skeleton = skeleton;

    // POIs first (index convention: start, victims..., gate).
    out.roadmap.start_idx = out.roadmap.addNode(start);
    out.roadmap.victim_idx.reserve(victims.size());
    for (const auto& v : victims)
        out.roadmap.victim_idx.push_back(out.roadmap.addNode(v));
    out.roadmap.gate_idx = out.roadmap.addNode(gate);

    // Skeleton cells become graph nodes.
    std::unordered_map<int, int> cell2node;
    cell2node.reserve(skeleton.size() * 2);
    for (int c : skeleton) {
        const int i = c % grid.nx;
        const int j = c / grid.nx;
        const Vec2 p = grid.world(i, j);
        const int node = out.roadmap.addNode(p);
        cell2node[c] = node;
        out.skeleton_pts.push_back(p);
    }

    // 8-connected skeleton cells become edges (verified with segmentClear).
    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int c : skeleton) {
        const int i = c % grid.nx;
        const int j = c / grid.nx;
        const int a = cell2node[c];
        for (int k = 0; k < 8; ++k) {
            const int ni = i + di[k], nj = j + dj[k];
            if (!grid.inBounds(ni, nj)) continue;
            const int nc = grid.index(ni, nj);
            auto it = cell2node.find(nc);
            if (it == cell2node.end()) continue;
            const int b = it->second;
            if (b <= a) continue;  // add each undirected edge once
            if (!segmentClear(out.roadmap.nodes[a], out.roadmap.nodes[b], map,
                              sample_res))
                continue;
            out.roadmap.adj[a].push_back(b);
            out.roadmap.adj[b].push_back(a);
        }
    }

    // Connect every POI to the nearest skeleton node reachable by a clear
    // segment; leave it isolated (OP distance +inf) if none is reachable.
    auto connect = [&](int poi_idx) {
        const Vec2 p = out.roadmap.nodes[poi_idx];
        int best = -1;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (const auto& kv : cell2node) {
            const int node = kv.second;
            const double d2 = dist2(p, out.roadmap.nodes[node]);
            if (d2 < best_d2 &&
                segmentClear(p, out.roadmap.nodes[node], map, sample_res)) {
                best_d2 = d2;
                best = node;
            }
        }
        if (best >= 0) {
            out.roadmap.adj[poi_idx].push_back(best);
            out.roadmap.adj[best].push_back(poi_idx);
        }
    };
    connect(out.roadmap.start_idx);
    for (int vi : out.roadmap.victim_idx) connect(vi);
    connect(out.roadmap.gate_idx);

    return out;
}

VoronoiRoadmap buildVoronoiRoadmap(const GeoMap& map, const Vec2& start,
                                   const std::vector<Vec2>& victims,
                                   const Vec2& gate, double res, int thin_iters,
                                   double sample_res) {
    const VoronoiGrid grid = buildDistanceTransform(map, res);
    const std::vector<int> skeleton =
        extractSkeleton(grid, map.clearance, thin_iters);
    return buildVoronoiGraph(grid, skeleton, map, start, victims, gate,
                             sample_res);
}

}  // namespace comb
