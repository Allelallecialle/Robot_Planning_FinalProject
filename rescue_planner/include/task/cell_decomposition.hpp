#pragma once

// ============================================================================
//  cell_decomposition.hpp  --  exact & approximate cell-decomposition roadmaps
// ----------------------------------------------------------------------------
//  This is the SECOND combinatorial roadmap strategy of the package (the first
//  being the visibility graph). It implements the cell-decomposition family
//  presented in the combinatorial-planning lecture:
//
//    * EXACT vertical / trapezoidal decomposition
//      (de Berg et al., "Computational Geometry", Ch. 6):
//      the free space is partitioned into non-overlapping vertical cells using
//      sweep lines placed at every obstacle-vertex x-coordinate (after
//      clearance inflation). Adjacent cells (whose free y-intervals overlap
//      across a shared sweep line) are connected in the roadmap. This
//      decomposition is COMPLETE: it finds a path through cell adjacency iff
//      one exists in the inflated free space.
//
//    * APPROXIMATE grid / occupancy decomposition
//      (Latombe, "Robot Motion Planning", Ch. 5):
//      the workspace is tiled by fixed-size square cells classified
//      FREE/OCCUPIED/MIXED by corner sampling; FREE cells are connected by
//      4-connectivity. Simpler, but NOT complete (narrow passages may lose
//      their free cell centres).
//
//  Both builders emit a `comb::Roadmap` (nodes + adjacency + POI indices) that
//  is fed to the shared tour_builder pipeline, exactly like the visibility
//  graph. All collision checking goes through the clearance-aware
//  geometry_utils (never raw obstacle geometry).
//
//  Design note (documented for the report): the clearance-expanded obstacles
//  are used IMPLICITLY through geometry_utils::pointInCollision; no explicit
//  Minkowski offset polygon is ever materialised.
// ============================================================================

#include <vector>

#include "task/tour_builder.hpp"   // comb::Roadmap alias + downstream pipeline
#include "utils/geometry_utils.hpp"

namespace comb {

/**
 * @brief A single vertical (trapezoidal) cell of the exact decomposition.
 *
 * The cell is the axis-aligned box [x_lo,x_hi] x [y_lo,y_hi] of the free
 * y-interval found at the probe abscissa of its slab. `node` is the index of
 * the cell's representative point inside the roadmap; `slab` is the index of
 * the vertical slab the cell belongs to (used for adjacency reasoning/tests).
 */
struct DecompCell {
    double x_lo = 0.0;
    double x_hi = 0.0;
    double y_lo = 0.0;
    double y_hi = 0.0;
    int    slab = -1;
    int    node = -1;

    /** @brief Representative point (slab-mid x, interval-mid y). */
    Vec2 rep() const {
        return {0.5 * (x_lo + x_hi), 0.5 * (y_lo + y_hi)};
    }
};

/**
 * @brief Full result of a cell-decomposition roadmap build.
 *
 * `roadmap` is the graph consumed by the tour builder; `cells` and `slab_x`
 * are retained for RViz visualisation and unit testing.
 */
struct CellDecomposition {
    Roadmap             roadmap;   ///< graph over cell reps + POIs
    std::vector<DecompCell> cells; ///< decomposition cells (exact) / grid cells (approx)
    std::vector<double> slab_x;    ///< sorted sweep-line x-coordinates (exact only)
};

/**
 * @brief Build the EXACT vertical (trapezoidal) cell-decomposition roadmap.
 *
 * Places sweep lines at the clearance-inflated left/right x of every obstacle
 * vertex (and of every circular obstacle's extent) plus the border, forms one
 * vertical slab per consecutive pair, extracts the free y-intervals at each
 * slab's mid abscissa, and connects overlapping cells of neighbouring slabs.
 * Every representative point is verified collision-free and every edge is
 * verified with segmentClear before being added. Points of interest are then
 * connected to the containing (or nearest reachable) cell.
 *
 * @param map        Clearance-aware geometry (obstacles, border, clearance).
 * @param start      Robot start position (POI 0).
 * @param victims    Victim centres (POIs 1..n, same order as the returned graph).
 * @param gate       Gate position (POI n+1).
 * @param sample_res Collision/interval sampling resolution [m]; the y-sweep is
 *                   done at sample_res/2 for finer interval boundaries.
 * @return CellDecomposition with the roadmap and the cells.
 * @complexity O(M) slabs x O(H/res) samples for the sweep, plus O(C^2) for the
 *             adjacency of C cells; C stays small for the expected scenarios.
 */
CellDecomposition buildCellDecomposition(const GeoMap& map, const Vec2& start,
                                         const std::vector<Vec2>& victims,
                                         const Vec2& gate, double sample_res);

/**
 * @brief Build the APPROXIMATE grid (occupancy) cell-decomposition roadmap.
 *
 * Tiles the border bounding box with square cells of side `cell_size`, marks a
 * cell FREE only when its centre and its four corners are all collision-free,
 * and connects FREE cells by 4-connectivity (edges verified with segmentClear).
 * NOT complete: narrow passages whose free centre disappears at this resolution
 * are lost. Marked clearly as approximate.
 *
 * @param map        Clearance-aware geometry.
 * @param start      Robot start position (POI 0).
 * @param victims    Victim centres (POIs 1..n).
 * @param gate       Gate position (POI n+1).
 * @param cell_size  Grid cell side length [m].
 * @param sample_res Collision sampling resolution for the connecting edges [m].
 * @return CellDecomposition with the roadmap and the FREE grid cells.
 * @complexity O(W*H / cell_size^2) cells; O(#cells) edges (4-connectivity).
 */
CellDecomposition buildApproxCellDecomposition(const GeoMap& map,
                                               const Vec2& start,
                                               const std::vector<Vec2>& victims,
                                               const Vec2& gate,
                                               double cell_size,
                                               double sample_res);

}  // namespace comb
