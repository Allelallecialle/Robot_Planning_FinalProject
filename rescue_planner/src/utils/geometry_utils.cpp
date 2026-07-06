#include "utils/geometry_utils.hpp"

#include <algorithm>
#include <limits>

namespace comb {

double cross(const Vec2& o, const Vec2& a, const Vec2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

bool pointInPolygon(const Vec2& p, const std::vector<Vec2>& poly) {
    const int n = static_cast<int>(poly.size());
    if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const double xi = poly[i].x, yi = poly[i].y;
        const double xj = poly[j].x, yj = poly[j].y;
        const bool intersect =
            ((yi > p.y) != (yj > p.y)) &&
            (p.x < (xj - xi) * (p.y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

double distPointToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) return dist(p, a);
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
    t = std::max(0.0, std::min(1.0, t));
    const Vec2 proj{a.x + t * dx, a.y + t * dy};
    return dist(p, proj);
}

double distPointToPolygonBoundary(const Vec2& p, const std::vector<Vec2>& poly) {
    const int n = static_cast<int>(poly.size());
    if (n == 0) return std::numeric_limits<double>::infinity();
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % n];
        best = std::min(best, distPointToSegment(p, a, b));
    }
    return best;
}

double polygonSignedArea(const std::vector<Vec2>& poly) {
    const int n = static_cast<int>(poly.size());
    double area = 0.0;
    for (int i = 0; i < n; ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % n];
        area += a.x * b.y - b.x * a.y;
    }
    return 0.5 * area;
}

bool pointInCollision(const Vec2& p, const GeoMap& map, double margin) {
    const double c = (margin < 0.0) ? map.clearance : margin;

    if (!map.border.empty()) {
        if (!pointInPolygon(p, map.border)) return true;
        if (distPointToPolygonBoundary(p, map.border) < c) return true;
    }

    for (const auto& obs : map.obstacles) {
        if (obs.is_circle) {
            if (dist(p, obs.center) < obs.radius + c) return true;
        } else {
            if (pointInPolygon(p, obs.verts)) return true;
            if (distPointToPolygonBoundary(p, obs.verts) < c) return true;
        }
    }
    return false;
}

bool segmentClear(const Vec2& a, const Vec2& b, const GeoMap& map,
                  double sample_res, double margin) {
    const double L = dist(a, b);
    const int steps = std::max(1, static_cast<int>(L / sample_res));
    // Sample open segment only; endpoints validated separately as graph nodes.
    for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        const Vec2 q{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
        if (pointInCollision(q, map, margin)) return false;
    }
    return true;
}

std::vector<Vec2> inflatedPolygonVertices(const std::vector<Vec2>& poly,
                                          double buffer, const GeoMap& map) {
    std::vector<Vec2> out;
    const int n = static_cast<int>(poly.size());
    if (n < 3) return out;

    const double area = polygonSignedArea(poly);
    const double sgn = (area > 0.0) ? 1.0 : -1.0;

    const double max_miter = 3.0 * buffer;

    for (int i = 0; i < n; ++i) {
        const Vec2& prev = poly[(i - 1 + n) % n];
        const Vec2& cur  = poly[i];
        const Vec2& next = poly[(i + 1) % n];

        Vec2 e1{cur.x - prev.x, cur.y - prev.y};
        Vec2 e2{next.x - cur.x, next.y - cur.y};
        const double l1 = std::hypot(e1.x, e1.y);
        const double l2 = std::hypot(e2.x, e2.y);
        if (l1 < 1e-9 || l2 < 1e-9) continue;
        e1.x /= l1; e1.y /= l1;
        e2.x /= l2; e2.y /= l2;

        // Outward normal of edge d is (d.y, -d.x) for CCW polygons.
        Vec2 n1{sgn * e1.y, -sgn * e1.x};
        Vec2 n2{sgn * e2.y, -sgn * e2.x};

        Vec2 bis{n1.x + n2.x, n1.y + n2.y};
        const double bl = std::hypot(bis.x, bis.y);
        if (bl < 1e-6) continue;
        bis.x /= bl; bis.y /= bl;

        double cosHalf = bis.x * n1.x + bis.y * n1.y;
        cosHalf = std::max(0.30, cosHalf);
        double miter = std::min(buffer / cosHalf, max_miter);

        Vec2 q{cur.x + bis.x * miter, cur.y + bis.y * miter};
        if (!pointInCollision(q, map)) out.push_back(q);
    }
    return out;
}

}  // namespace comb
