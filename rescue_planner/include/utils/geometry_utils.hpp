#pragma once

#include <vector>
#include <cmath>
#include <functional>

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

// Greedy line-of-sight simplification of a polyline, preserving its two
// endpoints. From the current vertex, jumps to the farthest later vertex
// still reachable through `segmentValid` (a caller-supplied collision test,
// so this works unchanged whether the underlying representation is a
// comb::GeoMap or a WorldModel). Used to collapse a dense roadmap path
// (grid/Voronoi cells, PRM/RRT/RRT*/RRG nodes) into the fewest safe
// straight-line legs before Dubins fitting, instead of stitching an arc
// between every intermediate roadmap node.
//
// IMPORTANT: call this per POI-to-POI leg, not on a whole multi-victim tour
// at once -- it only guarantees the two endpoints of `poly` survive, so an
// intermediate victim stop must be a leg endpoint of its own call, or the
// shortcut search could jump straight over it.
std::vector<Vec2> simplifyLineOfSight(
    const std::vector<Vec2>& poly,
    const std::function<bool(const Vec2&, const Vec2&)>& segmentValid);

}  // namespace comb
