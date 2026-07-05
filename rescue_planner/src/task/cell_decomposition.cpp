#include "task/cell_decomposition.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>

namespace comb {

namespace {

// Axis-aligned bounding box of a polygon.
struct BBox {
    double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
    bool valid = false;
};

BBox borderBBox(const GeoMap& map) {
    BBox b;
    if (map.border.empty()) return b;
    b.min_x = b.max_x = map.border.front().x;
    b.min_y = b.max_y = map.border.front().y;
    for (const auto& p : map.border) {
        b.min_x = std::min(b.min_x, p.x);
        b.max_x = std::max(b.max_x, p.x);
        b.min_y = std::min(b.min_y, p.y);
        b.max_y = std::max(b.max_y, p.y);
    }
    b.valid = true;
    return b;
}

// Insert a clamped, de-duplicated x sweep event.
void addEvent(std::vector<double>& xs, double x, double lo, double hi) {
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    xs.push_back(x);
}

// Connect a point of interest to the roadmap: to every cell whose free
// y-interval contains it in its own slab (via a clear segment), and, if none is
// directly reachable, to the nearest cell by Euclidean distance with a
// segmentClear fallback. Adds undirected adjacency to `g`.
void connectPoi(Roadmap& g, int poi_idx, const GeoMap& map,
                const std::vector<DecompCell>& cells,
                const std::vector<double>& slab_x, double sample_res) {
    const Vec2 p = g.nodes[poi_idx];

    // Slab that contains p.x (the interval [slab_x[s], slab_x[s+1]]).
    int slab = -1;
    for (std::size_t s = 0; s + 1 < slab_x.size(); ++s) {
        if (p.x >= slab_x[s] - 1e-9 && p.x <= slab_x[s + 1] + 1e-9) {
            slab = static_cast<int>(s);
            break;
        }
    }

    bool connected = false;
    for (const auto& c : cells) {
        if (c.slab != slab) continue;
        if (p.y < c.y_lo - 1e-9 || p.y > c.y_hi + 1e-9) continue;
        if (segmentClear(p, g.nodes[c.node], map, sample_res)) {
            g.adj[poi_idx].push_back(c.node);
            g.adj[c.node].push_back(poi_idx);
            connected = true;
        }
    }
    if (connected) return;

    // Fallback: nearest representative reachable by a clear straight segment.
    int best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto& c : cells) {
        const double d2 = dist2(p, g.nodes[c.node]);
        if (d2 < best_d2 && segmentClear(p, g.nodes[c.node], map, sample_res)) {
            best_d2 = d2;
            best = c.node;
        }
    }
    if (best >= 0) {
        g.adj[poi_idx].push_back(best);
        g.adj[best].push_back(poi_idx);
    }
}

}  // namespace

CellDecomposition buildCellDecomposition(const GeoMap& map, const Vec2& start,
                                         const std::vector<Vec2>& victims,
                                         const Vec2& gate, double sample_res) {
    CellDecomposition out;
    const double r_c = map.clearance;

    const BBox bb = borderBBox(map);
    if (!bb.valid) return out;  // no border => nothing to decompose

    // ---------- POIs always become nodes (index convention: start,victims,gate)
    out.roadmap.start_idx = out.roadmap.addNode(start);
    out.roadmap.victim_idx.reserve(victims.size());
    for (const auto& v : victims)
        out.roadmap.victim_idx.push_back(out.roadmap.addNode(v));
    out.roadmap.gate_idx = out.roadmap.addNode(gate);

    // ---------- Step 1: collect sweep-line x events ---------------------------
    std::vector<double>& xs = out.slab_x;
    addEvent(xs, bb.min_x, bb.min_x, bb.max_x);
    addEvent(xs, bb.max_x, bb.min_x, bb.max_x);
    for (const auto& obs : map.obstacles) {
        if (obs.is_circle) {
            addEvent(xs, obs.center.x - obs.radius - r_c, bb.min_x, bb.max_x);
            addEvent(xs, obs.center.x + obs.radius + r_c, bb.min_x, bb.max_x);
        } else {
            for (const auto& v : obs.verts) {
                addEvent(xs, v.x - r_c, bb.min_x, bb.max_x);
                addEvent(xs, v.x + r_c, bb.min_x, bb.max_x);
            }
        }
    }
    std::sort(xs.begin(), xs.end());
    // De-duplicate nearly-coincident events (keep slabs non-degenerate).
    std::vector<double> uniq;
    for (double x : xs) {
        if (uniq.empty() || x - uniq.back() > std::max(1e-6, sample_res * 0.5))
            uniq.push_back(x);
    }
    xs.swap(uniq);

    // ---------- Steps 2-3: free y-intervals per slab --------------------------
    const double dy = std::max(sample_res * 0.5, 1e-3);
    for (std::size_t s = 0; s + 1 < xs.size(); ++s) {
        const double x_lo = xs[s];
        const double x_hi = xs[s + 1];
        const double x_mid = 0.5 * (x_lo + x_hi);

        bool in_run = false;
        double run_lo = 0.0, run_lo_prev = 0.0;
        double y = bb.min_y;
        // Sweep inclusive of the top border; a trailing !free closes the run.
        while (y <= bb.max_y + 1e-9) {
            const bool free = !pointInCollision({x_mid, y}, map);
            if (free && !in_run) {
                in_run = true;
                run_lo = y;
            } else if (!free && in_run) {
                in_run = false;
                const double y_lo = run_lo;
                const double y_hi = run_lo_prev;  // last free sample
                if (y_hi - y_lo >= 2.0 * r_c) {
                    DecompCell c;
                    c.x_lo = x_lo; c.x_hi = x_hi;
                    c.y_lo = y_lo; c.y_hi = y_hi;
                    c.slab = static_cast<int>(s);
                    const Vec2 rep{x_mid, 0.5 * (y_lo + y_hi)};
                    if (!pointInCollision(rep, map)) {
                        c.node = out.roadmap.addNode(rep);
                        out.cells.push_back(c);
                    }
                }
            }
            if (free) run_lo_prev = y;
            y += dy;
        }
        if (in_run) {  // free run reaching the top border
            const double y_lo = run_lo;
            const double y_hi = run_lo_prev;
            if (y_hi - y_lo >= 2.0 * r_c) {
                DecompCell c;
                c.x_lo = x_lo; c.x_hi = x_hi;
                c.y_lo = y_lo; c.y_hi = y_hi;
                c.slab = static_cast<int>(s);
                const Vec2 rep{x_mid, 0.5 * (y_lo + y_hi)};
                if (!pointInCollision(rep, map)) {
                    c.node = out.roadmap.addNode(rep);
                    out.cells.push_back(c);
                }
            }
        }
    }

    // ---------- Step 4: cell adjacency across neighbouring slabs --------------
    for (std::size_t i = 0; i < out.cells.size(); ++i) {
        for (std::size_t j = i + 1; j < out.cells.size(); ++j) {
            const DecompCell& a = out.cells[i];
            const DecompCell& b = out.cells[j];
            if (std::abs(a.slab - b.slab) != 1) continue;  // only neighbours
            const double overlap =
                std::min(a.y_hi, b.y_hi) - std::max(a.y_lo, b.y_lo);
            if (overlap < sample_res) continue;
            if (!segmentClear(out.roadmap.nodes[a.node],
                              out.roadmap.nodes[b.node], map, sample_res))
                continue;
            out.roadmap.adj[a.node].push_back(b.node);
            out.roadmap.adj[b.node].push_back(a.node);
        }
    }

    // ---------- Step 5: connect the POIs --------------------------------------
    connectPoi(out.roadmap, out.roadmap.start_idx, map, out.cells, xs, sample_res);
    for (int vi : out.roadmap.victim_idx)
        connectPoi(out.roadmap, vi, map, out.cells, xs, sample_res);
    connectPoi(out.roadmap, out.roadmap.gate_idx, map, out.cells, xs, sample_res);

    return out;
}

CellDecomposition buildApproxCellDecomposition(const GeoMap& map,
                                               const Vec2& start,
                                               const std::vector<Vec2>& victims,
                                               const Vec2& gate,
                                               double cell_size,
                                               double sample_res) {
    CellDecomposition out;
    const BBox bb = borderBBox(map);
    if (!bb.valid || cell_size <= 0.0) return out;

    out.roadmap.start_idx = out.roadmap.addNode(start);
    out.roadmap.victim_idx.reserve(victims.size());
    for (const auto& v : victims)
        out.roadmap.victim_idx.push_back(out.roadmap.addNode(v));
    out.roadmap.gate_idx = out.roadmap.addNode(gate);

    const int nx =
        std::max(1, static_cast<int>(std::ceil((bb.max_x - bb.min_x) / cell_size)));
    const int ny =
        std::max(1, static_cast<int>(std::ceil((bb.max_y - bb.min_y) / cell_size)));

    // grid[i][j] -> node index of a FREE cell, or -1.
    std::vector<std::vector<int>> grid(nx, std::vector<int>(ny, -1));
    const double h = 0.5 * cell_size;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const double cx = bb.min_x + (i + 0.5) * cell_size;
            const double cy = bb.min_y + (j + 0.5) * cell_size;
            // FREE only when the centre AND all four corners are clear.
            const Vec2 probes[5] = {{cx, cy},
                                    {cx - h, cy - h}, {cx + h, cy - h},
                                    {cx - h, cy + h}, {cx + h, cy + h}};
            bool all_free = true;
            for (const auto& q : probes) {
                if (pointInCollision(q, map)) { all_free = false; break; }
            }
            if (all_free) {
                DecompCell c;
                c.x_lo = cx - h; c.x_hi = cx + h;
                c.y_lo = cy - h; c.y_hi = cy + h;
                c.slab = i;
                c.node = out.roadmap.addNode({cx, cy});
                grid[i][j] = c.node;
                out.cells.push_back(c);
            }
        }
    }

    // 4-connectivity edges between neighbouring FREE cells.
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const int a = grid[i][j];
            if (a < 0) continue;
            const int ni[2] = {i + 1, i};
            const int nj[2] = {j, j + 1};
            for (int d = 0; d < 2; ++d) {
                if (ni[d] >= nx || nj[d] >= ny) continue;
                const int b = grid[ni[d]][nj[d]];
                if (b < 0) continue;
                if (!segmentClear(out.roadmap.nodes[a], out.roadmap.nodes[b],
                                  map, sample_res))
                    continue;
                out.roadmap.adj[a].push_back(b);
                out.roadmap.adj[b].push_back(a);
            }
        }
    }

    // Connect POIs to the nearest reachable FREE cell centre.
    auto connect = [&](int poi_idx) {
        const Vec2 p = out.roadmap.nodes[poi_idx];
        int best = -1;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (const auto& c : out.cells) {
            const double d2 = dist2(p, out.roadmap.nodes[c.node]);
            if (d2 < best_d2 &&
                segmentClear(p, out.roadmap.nodes[c.node], map, sample_res)) {
                best_d2 = d2;
                best = c.node;
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

}  // namespace comb
