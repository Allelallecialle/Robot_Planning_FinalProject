#pragma once

// ============================================================================
//  geometry_utils.hpp  --  geometry primitives for the COMBINATORIAL planner
// ----------------------------------------------------------------------------
//  These utilities are intentionally kept SEPARATE from the teammate's
//  collision_checker.{hpp,cpp}: that checker only tests against the raw
//  (non-inflated) obstacles and therefore is not safe for a robot with a
//  physical footprint. The combinatorial strategy needs *clearance-aware*
//  geometry (Minkowski inflation by robot_radius + safety margin), so we
//  provide our own, self-contained helpers here under namespace `comb`.
//
//  Design choice (documented for the report):
//  -----------------------------------------
//  We do NOT explicitly build offset (inflated) polygons for collision
//  checking. Instead we test a query point against the ORIGINAL obstacle and
//  flag a collision when the point is inside the obstacle OR closer than
//  `clearance` to its boundary. This is *exactly* the Minkowski sum of the
//  obstacle with a disk of radius `clearance`, and it works for arbitrary
//  (convex or non-convex) polygons and circles uniformly, with no offset-
//  polygon bookkeeping. The only place we need explicit offset *vertices* is
//  to generate visibility-graph waypoints around obstacle corners
//  (`inflatedPolygonVertices`).
// ============================================================================

#include <vector>
#include <cmath>

namespace comb {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

// Unified obstacle representation built from the live /obstacles topic.
struct Obstacle {
    bool is_circle = false;
    Vec2 center;                 // valid when is_circle == true
    double radius = 0.0;         // valid when is_circle == true
    std::vector<Vec2> verts;     // valid when is_circle == false (polygon)
};

// The full geometric description of the planning problem.
struct GeoMap {
    std::vector<Obstacle> obstacles;  // ORIGINAL (non-inflated) obstacles
    std::vector<Vec2>     border;     // ORIGINAL map border polygon (CW or CCW)
    double clearance = 0.30;          // robot_radius + safety margin [m]
};

// ---- basic vector helpers ----
inline double dist(const Vec2& a, const Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}
inline double dist2(const Vec2& a, const Vec2& b) {
    const double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double cross(const Vec2& o, const Vec2& a, const Vec2& b);

// ---- polygon predicates ----
bool   pointInPolygon(const Vec2& p, const std::vector<Vec2>& poly);
double distPointToSegment(const Vec2& p, const Vec2& a, const Vec2& b);
double distPointToPolygonBoundary(const Vec2& p, const std::vector<Vec2>& poly);
double polygonSignedArea(const std::vector<Vec2>& poly);

// ---- clearance-aware collision test ----
// Returns true if the configuration `p` is in collision with the inflated
// world: i.e. outside the border (minus clearance), or within `clearance`
// of any obstacle (Minkowski inflation). An explicit `margin` override can be
// passed for the curved-arc re-check; if margin < 0 we use map.clearance.
bool pointInCollision(const Vec2& p, const GeoMap& map, double margin = -1.0);

// Samples the open segment (a,b) and returns true if every interior sample is
// collision-free (clearance-aware). Endpoints are assumed pre-validated.
bool segmentClear(const Vec2& a, const Vec2& b, const GeoMap& map,
                  double sample_res = 0.05, double margin = -1.0);

// ---- visibility-graph node generation ----
// Miter-offsets each polygon vertex OUTWARD by `buffer` along the corner
// bisector so the resulting point sits in free space around the corner. The
// miter length is clamped to avoid spikes at very sharp corners. Returns only
// the offset points that are themselves collision-free in `map`.
std::vector<Vec2> inflatedPolygonVertices(const std::vector<Vec2>& poly,
                                          double buffer, const GeoMap& map);

}  // namespace comb
