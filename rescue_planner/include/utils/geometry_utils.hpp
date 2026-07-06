#pragma once

#include <vector>
#include <cmath>

namespace comb {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Obstacle {
    bool is_circle = false;
    Vec2 center;
    double radius = 0.0;
    std::vector<Vec2> verts;
};

struct GeoMap {
    std::vector<Obstacle> obstacles;
    std::vector<Vec2>     border;
    double clearance = 0.30;
};

inline double dist(const Vec2& a, const Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}
inline double dist2(const Vec2& a, const Vec2& b) {
    const double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double cross(const Vec2& o, const Vec2& a, const Vec2& b);

bool   pointInPolygon(const Vec2& p, const std::vector<Vec2>& poly);
double distPointToSegment(const Vec2& p, const Vec2& a, const Vec2& b);
double distPointToPolygonBoundary(const Vec2& p, const std::vector<Vec2>& poly);
double polygonSignedArea(const std::vector<Vec2>& poly);

// Clearance-aware collision (Minkowski inflation via distance test, not offset polygons).
bool pointInCollision(const Vec2& p, const GeoMap& map, double margin = -1.0);
bool segmentClear(const Vec2& a, const Vec2& b, const GeoMap& map,
                  double sample_res = 0.05, double margin = -1.0);

std::vector<Vec2> inflatedPolygonVertices(const std::vector<Vec2>& poly,
                                          double buffer, const GeoMap& map);

}  // namespace comb
