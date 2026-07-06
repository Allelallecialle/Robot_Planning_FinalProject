#pragma once

#include <cstdint>
#include <vector>

#include "task/tour_builder.hpp"
#include "utils/geometry_utils.hpp"

namespace comb {

struct VoronoiGrid {
    int    nx = 0;
    int    ny = 0;
    double res = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;

    std::vector<std::uint8_t> occupied;
    std::vector<double>       dist;
    std::vector<int>          obstacle_id;

    int index(int i, int j) const { return j * nx + i; }
    bool inBounds(int i, int j) const {
        return i >= 0 && i < nx && j >= 0 && j < ny;
    }
    Vec2 world(int i, int j) const {
        return {origin_x + i * res, origin_y + j * res};
    }
};

struct VoronoiRoadmap {
    Roadmap           roadmap;
    VoronoiGrid       grid;
    std::vector<int>  skeleton;
    std::vector<Vec2> skeleton_pts;
};

VoronoiGrid buildDistanceTransform(const GeoMap& map, double res);
std::vector<int> extractSkeleton(const VoronoiGrid& grid, double clearance,
                                 int thin_iters);
VoronoiRoadmap buildVoronoiGraph(const VoronoiGrid& grid,
                                 const std::vector<int>& skeleton,
                                 const GeoMap& map, const Vec2& start,
                                 const std::vector<Vec2>& victims,
                                 const Vec2& gate, double sample_res);
VoronoiRoadmap buildVoronoiRoadmap(const GeoMap& map, const Vec2& start,
                                   const std::vector<Vec2>& victims,
                                   const Vec2& gate, double res, int thin_iters,
                                   double sample_res);

}  // namespace comb
