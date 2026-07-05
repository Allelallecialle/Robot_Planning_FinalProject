#pragma once

// ============================================================================
//  voronoi_roadmap.hpp  --  maximum-clearance (Generalised Voronoi) roadmap
// ----------------------------------------------------------------------------
//  Third combinatorial roadmap strategy of the package, implementing the
//  maximum-clearance roadmap of the combinatorial-planning lecture
//  (Choset et al., "Principles of Robot Motion", Ch. 5). Paths on the
//  Generalised Voronoi Diagram (GVD) stay equidistant from two or more
//  obstacle boundaries, i.e. they are the paths of maximum local clearance.
//
//  We build a DISCRETE approximation of the GVD over a uniform grid:
//    1. occupancy grid              : a cell is OCCUPIED iff it lies inside an
//                                     obstacle or outside the border (the border
//                                     wall is obstacle id 0). NOTE the occupancy
//                                     is computed against the RAW obstacles (no
//                                     clearance inflation) so that the distance
//                                     transform below equals the TRUE clearance
//                                     to the nearest obstacle boundary.
//    2. distance transform          : for every FREE cell, the Euclidean
//                                     distance (in metres) to the nearest
//                                     OCCUPIED cell, plus the id of that nearest
//                                     obstacle (feature transform, propagated by
//                                     a min-heap Dijkstra with 8-connectivity).
//    3. GVD ridge extraction        : a FREE cell is a ridge candidate if any of
//                                     its 8 neighbours has a DIFFERENT nearest-
//                                     obstacle id (it is equidistant from two
//                                     obstacles).
//    4. prune + thin                : discard candidates whose clearance
//                                     (== dist) is below the robot clearance
//                                     radius r_c (the robot could not fit), then
//                                     Zhang-Suen thin the ridge to a 1-cell
//                                     skeleton.
//    5. build the graph             : skeleton cells become nodes, 8-connected
//                                     skeleton cells become edges.
//    6. connect the POIs            : each POI is joined to the nearest skeleton
//                                     node reachable by a clear segment.
//
//  Properties (documented for the report):
//    * The GVD path is maximally far from obstacles (maximum safety) but is
//      generally LONGER than the visibility-graph shortest path -- the central
//      safety/length trade-off to compare in the report.
//    * The grid resolution `voronoi_res` trades approximation quality against
//      computation time.
//    * This is an APPROXIMATION of the true GVD and is NOT exact-complete: if
//      the discrete skeleton does not connect two POIs, no path is returned.
// ============================================================================

#include <cstdint>
#include <vector>

#include "task/tour_builder.hpp"   // comb::Roadmap alias + downstream pipeline
#include "utils/geometry_utils.hpp"

namespace comb {

/**
 * @brief Uniform occupancy + distance-transform grid over the padded workspace.
 *
 * The grid is padded by a few cells beyond the border bounding box so that the
 * exterior cells act as border (id 0) obstacle seeds for the distance
 * transform. Cell (i,j) has world centre `world(i,j)` and linear index
 * `index(i,j)`.
 */
struct VoronoiGrid {
    int    nx = 0;
    int    ny = 0;
    double res = 0.05;      ///< cell size [m]
    double origin_x = 0.0;  ///< world x of cell (0,*)
    double origin_y = 0.0;  ///< world y of cell (*,0)

    std::vector<std::uint8_t> occupied;  ///< 1 == OCCUPIED, 0 == FREE
    std::vector<double>       dist;       ///< clearance to nearest obstacle [m]
    std::vector<int>          obstacle_id;///< nearest obstacle id (0 == border)

    int index(int i, int j) const { return j * nx + i; }
    bool inBounds(int i, int j) const {
        return i >= 0 && i < nx && j >= 0 && j < ny;
    }
    Vec2 world(int i, int j) const {
        return {origin_x + i * res, origin_y + j * res};
    }
};

/**
 * @brief Full result of a Voronoi roadmap build (graph + artefacts for viz/tests).
 */
struct VoronoiRoadmap {
    Roadmap           roadmap;   ///< graph over skeleton cells + POIs
    VoronoiGrid       grid;      ///< occupancy + distance transform
    std::vector<int>  skeleton;  ///< linear grid indices of the skeleton cells
    std::vector<Vec2> skeleton_pts;  ///< skeleton cell world centres (for viz)
};

/**
 * @brief Build the occupancy grid and its Euclidean distance/feature transform.
 *
 * A cell is OCCUPIED iff its centre lies outside the border (id 0) or inside an
 * obstacle (id = obstacle index + 1); OCCUPIED cells get dist 0. Every FREE
 * cell receives the Euclidean distance (metres) to, and the obstacle id of, its
 * nearest OCCUPIED cell, computed by a min-heap feature-propagation Dijkstra
 * with 8-connectivity.
 *
 * @param map Clearance-aware geometry (raw obstacles are used for occupancy).
 * @param res Grid resolution [m].
 * @return The populated VoronoiGrid.
 * @complexity O(N log N) for N grid cells (min-heap Dijkstra).
 */
VoronoiGrid buildDistanceTransform(const GeoMap& map, double res);

/**
 * @brief Extract and thin the GVD skeleton from a distance/feature transform.
 *
 * A FREE cell is a ridge candidate if any 8-neighbour has a different nearest-
 * obstacle id. Candidates whose clearance (dist) is below `clearance` are
 * discarded (the robot could not fit). The remaining ridge is thinned to a
 * 1-cell-wide skeleton by up to `thin_iters` Zhang-Suen iterations (which
 * preserves connectivity and endpoints).
 *
 * @param grid       Grid produced by buildDistanceTransform.
 * @param clearance  Robot clearance radius r_c [m]; skeleton cells keep dist >= r_c.
 * @param thin_iters Maximum Zhang-Suen thinning iterations.
 * @return Linear grid indices of the skeleton cells (possibly empty).
 * @complexity O(thin_iters * N) for N grid cells.
 */
std::vector<int> extractSkeleton(const VoronoiGrid& grid, double clearance,
                                 int thin_iters);

/**
 * @brief Build the Voronoi roadmap graph from a skeleton and connect the POIs.
 *
 * Each skeleton cell becomes a graph node (world centre); 8-connected skeleton
 * cells become edges (verified with segmentClear). Each POI is connected to the
 * nearest skeleton node reachable by a clear straight segment; if none is
 * reachable the POI is left isolated (its OP distances become +infinity).
 *
 * @param grid       Grid produced by buildDistanceTransform.
 * @param skeleton   Skeleton cell indices from extractSkeleton.
 * @param map        Clearance-aware geometry (for the segmentClear checks).
 * @param start      Robot start position (POI 0).
 * @param victims    Victim centres (POIs 1..n).
 * @param gate       Gate position (POI n+1).
 * @param sample_res Collision sampling resolution for the edges [m].
 * @return VoronoiRoadmap with the graph, grid and skeleton artefacts.
 * @complexity O(S) edges for S skeleton cells, plus O((n+2) * S) POI hookup.
 */
VoronoiRoadmap buildVoronoiGraph(const VoronoiGrid& grid,
                                 const std::vector<int>& skeleton,
                                 const GeoMap& map, const Vec2& start,
                                 const std::vector<Vec2>& victims,
                                 const Vec2& gate, double sample_res);

/**
 * @brief Convenience: run steps 1-6 (distance transform, skeleton, graph).
 *
 * @param map        Clearance-aware geometry.
 * @param start      Robot start position (POI 0).
 * @param victims    Victim centres (POIs 1..n).
 * @param gate       Gate position (POI n+1).
 * @param res        Grid resolution [m].
 * @param thin_iters Maximum thinning iterations.
 * @param sample_res Collision sampling resolution for the edges [m].
 * @return The complete VoronoiRoadmap.
 */
VoronoiRoadmap buildVoronoiRoadmap(const GeoMap& map, const Vec2& start,
                                   const std::vector<Vec2>& victims,
                                   const Vec2& gate, double res, int thin_iters,
                                   double sample_res);

}  // namespace comb
