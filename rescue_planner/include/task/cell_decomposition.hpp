#pragma once

#include <vector>

#include "task/tour_builder.hpp"
#include "utils/geometry_utils.hpp"

namespace comb {

struct DecompCell {
    double x_lo = 0.0;
    double x_hi = 0.0;
    double y_lo = 0.0;
    double y_hi = 0.0;
    int    slab = -1;
    int    node = -1;

    Vec2 rep() const {
        return {0.5 * (x_lo + x_hi), 0.5 * (y_lo + y_hi)};
    }
};

struct CellDecomposition {
    Roadmap             roadmap;
    std::vector<DecompCell> cells;
    std::vector<double> slab_x;
};

CellDecomposition buildCellDecomposition(const GeoMap& map, const Vec2& start,
                                         const std::vector<Vec2>& victims,
                                         const Vec2& gate, double sample_res);
CellDecomposition buildApproxCellDecomposition(const GeoMap& map,
                                               const Vec2& start,
                                               const std::vector<Vec2>& victims,
                                               const Vec2& gate,
                                               double cell_size,
                                               double sample_res);

}  // namespace comb
