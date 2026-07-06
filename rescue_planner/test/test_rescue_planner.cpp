#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

#include "utils/geometry_utils.hpp"
#include "task/visibility_graph.hpp"
#include "task/cell_decomposition.hpp"
#include "task/voronoi_roadmap.hpp"
#include "task/orienteering.hpp"
#include "task/tour_builder.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"
#include "utils/planning_budget.hpp"

#include "collision_checker.hpp"
#include "world_model.hpp"

namespace {

constexpr double kClearance = 0.30;
const comb::Vec2 kStart{1.0, 1.0};
const comb::Vec2 kVictim{2.0, 7.0};
const comb::Vec2 kGate{8.0, 2.0};

// Fixed course-standard rescue budget (30 s at v_max=0.30 m/s, derated).
constexpr int    kTestTimeoutSec    = 30;
constexpr double   kTestVmax          = 0.30;
constexpr double   kTestDubinsSafety  = 0.85;
constexpr double   kTestDmax          =
    comb::distanceBudget(kTestTimeoutSec, kTestVmax, kTestDubinsSafety);

constexpr int    kVictimTimeoutSec    = 60;
constexpr double kVictimBudgetDmax    =
    comb::distanceBudget(kVictimTimeoutSec, kTestVmax, kTestDubinsSafety);

comb::GeoMap makeGeoMap(double clearance = kClearance) {
    comb::GeoMap map;
    map.clearance = clearance;
    map.border = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};

    comb::Obstacle square;
    square.is_circle = false;
    square.verts = {{4.0, 4.0}, {6.0, 4.0}, {6.0, 6.0}, {4.0, 6.0}};
    map.obstacles.push_back(square);

    comb::Obstacle circle;
    circle.is_circle = true;
    circle.center = {3.0, 3.0};
    circle.radius = 0.5;
    map.obstacles.push_back(circle);
    return map;
}

std::vector<comb::Vec2> victims() { return {kVictim}; }

geometry_msgs::Point32 pt(double x, double y) {
    geometry_msgs::Point32 p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = 0.0f;
    return p;
}

WorldModel makeWorldModel() {
    WorldModel w;
    w.borders.points = {pt(0, 0), pt(10, 0), pt(10, 10), pt(0, 10)};

    WorldModel::Obstacle box;
    box.polygon.points = {pt(4, 4), pt(6, 4), pt(6, 6), pt(4, 6)};
    box.radius = 0.0;
    w.obstacles.push_back(box);

    WorldModel::Obstacle cyl;
    cyl.polygon.points = {pt(3, 3)};
    cyl.radius = 0.5;
    w.obstacles.push_back(cyl);

    WorldModel::Victim v;
    v.x = kVictim.x; v.y = kVictim.y; v.radius = 0.5; v.value = 10.0;
    w.victims.push_back(v);

    w.start.x = kStart.x; w.start.y = kStart.y; w.start.yaw = 0.0;
    return w;
}

bool polylineClear(const comb::VisibilityGraph& g, const std::vector<int>& path,
                   const comb::GeoMap& map, double res) {
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const comb::Vec2 a = g.nodes[path[i]];
        const comb::Vec2 b = g.nodes[path[i + 1]];
        const double L = comb::dist(a, b);
        const int steps = std::max(1, static_cast<int>(L / res));
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            const comb::Vec2 q{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
            if (comb::pointInCollision(q, map)) return false;
        }
    }
    return true;
}

}  // namespace

TEST(GeometryUtils, PointInPolygon) {
    const comb::GeoMap map = makeGeoMap();
    EXPECT_TRUE(comb::pointInPolygon({5.0, 5.0}, map.border));
    EXPECT_FALSE(comb::pointInPolygon({11.0, 11.0}, map.border));
    (void)comb::pointInPolygon({0.0, 5.0}, map.border);
}

TEST(GeometryUtils, PointInCollisionClearance) {
    const comb::GeoMap map = makeGeoMap(0.30);
    EXPECT_TRUE(comb::pointInCollision({0.1, 0.1}, map));
    EXPECT_FALSE(comb::pointInCollision({1.0, 1.0}, map));
    EXPECT_TRUE(comb::pointInCollision({5.0, 5.0}, map));
    EXPECT_TRUE(comb::pointInCollision({4.4, 5.0}, map));
    EXPECT_TRUE(comb::pointInCollision({3.0, 3.0}, map));
    EXPECT_TRUE(comb::pointInCollision({3.0, 3.7}, map));
    EXPECT_FALSE(comb::pointInCollision({3.0, 4.0}, map));
}

TEST(GeometryUtils, SegmentClear) {
    const comb::GeoMap map = makeGeoMap();
    EXPECT_TRUE(comb::segmentClear({1.0, 1.0}, {2.0, 2.0}, map, 0.02));
    EXPECT_FALSE(comb::segmentClear({1.0, 1.0}, {5.0, 5.0}, map, 0.02));
}

TEST(GeometryUtils, InflatedPolygonVerticesAreFree) {
    const comb::GeoMap map = makeGeoMap();
    const std::vector<comb::Vec2> square = {
        {4.0, 4.0}, {6.0, 4.0}, {6.0, 6.0}, {4.0, 6.0}};
    const double buffer = map.clearance + 0.10;
    const std::vector<comb::Vec2> infl =
        comb::inflatedPolygonVertices(square, buffer, map);
    EXPECT_FALSE(infl.empty());
    for (const auto& q : infl) EXPECT_FALSE(comb::pointInCollision(q, map));
}

TEST(VisibilityGraph, ContainsPoisAndClearEdges) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VisibilityGraph g = comb::buildVisibilityGraph(
        map, kStart, victims(), kGate, map.clearance + 0.10, 0.02);

    ASSERT_GE(static_cast<int>(g.nodes.size()), 3);
    EXPECT_GE(g.start_idx, 0);
    EXPECT_GE(g.gate_idx, 0);
    ASSERT_EQ(g.victim_idx.size(), 1u);

    for (int i = 0; i < static_cast<int>(g.adj.size()); ++i)
        for (int j : g.adj[i])
            if (j > i)
                EXPECT_TRUE(comb::segmentClear(g.nodes[i], g.nodes[j], map, 0.02))
                    << "edge " << i << "-" << j;
}

TEST(VisibilityGraph, DijkstraAndReconstruct) {
    comb::VisibilityGraph g;
    const int a = g.addNode({0.0, 0.0});
    const int b = g.addNode({1.0, 0.0});
    const int c = g.addNode({5.0, 5.0});
    g.adj[a].push_back(b);
    g.adj[b].push_back(a);

    std::vector<double> dist;
    std::vector<int> prev;
    comb::dijkstra(g, a, dist, prev);
    EXPECT_DOUBLE_EQ(dist[a], 0.0);
    EXPECT_NEAR(dist[b], 1.0, 1e-9);
    EXPECT_TRUE(std::isinf(dist[c]));

    const std::vector<int> path = comb::reconstructPath(a, b, prev);
    ASSERT_EQ(path.size(), 2u);
    EXPECT_EQ(path.front(), a);
    EXPECT_EQ(path.back(), b);
    EXPECT_TRUE(comb::reconstructPath(a, c, prev).empty());
}

TEST(VisibilityGraph, ShortestPathAvoidsObstacles) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VisibilityGraph g = comb::buildVisibilityGraph(
        map, kStart, victims(), kGate, map.clearance + 0.10, 0.02);

    std::vector<double> dist;
    std::vector<int> prev;
    comb::dijkstra(g, g.start_idx, dist, prev);
    ASSERT_TRUE(std::isfinite(dist[g.gate_idx]));

    const std::vector<int> path =
        comb::reconstructPath(g.start_idx, g.gate_idx, prev);
    ASSERT_GE(path.size(), 2u);
    EXPECT_TRUE(polylineClear(g, path, map, 0.02));
    EXPECT_LT(dist[g.gate_idx], kTestDmax);
}

TEST(CellDecomposition, BuildAndFreeCells) {
    const comb::GeoMap map = makeGeoMap();
    const comb::CellDecomposition d =
        comb::buildCellDecomposition(map, kStart, victims(), kGate, 0.05);

    ASSERT_FALSE(d.cells.empty());
    for (const auto& c : d.cells)
        EXPECT_FALSE(comb::pointInCollision(d.roadmap.nodes[c.node], map));
}

TEST(CellDecomposition, AdjacencyBetweenNeighbouringSlabs) {
    const comb::GeoMap map = makeGeoMap();
    const comb::CellDecomposition d =
        comb::buildCellDecomposition(map, kStart, victims(), kGate, 0.05);

    std::vector<int> node2slab(d.roadmap.nodes.size(), -1);
    for (const auto& c : d.cells) node2slab[c.node] = c.slab;

    for (const auto& c : d.cells) {
        for (int nb : d.roadmap.adj[c.node]) {
            if (node2slab[nb] < 0) continue;
            EXPECT_EQ(std::abs(node2slab[c.node] - node2slab[nb]), 1);
        }
    }
}

TEST(CellDecomposition, PoiReachabilityAndDistances) {
    const comb::GeoMap map = makeGeoMap();
    const comb::CellDecomposition d =
        comb::buildCellDecomposition(map, kStart, victims(), kGate, 0.05);
    const comb::VisibilityGraph& g = d.roadmap;

    std::vector<double> dist;
    std::vector<int> prev;
    comb::dijkstra(g, g.start_idx, dist, prev);
    EXPECT_TRUE(std::isfinite(dist[g.gate_idx]));
    ASSERT_EQ(g.victim_idx.size(), 1u);
    EXPECT_TRUE(std::isfinite(dist[g.victim_idx[0]]));

    // Cell roadmap exceeds 30 s budget on this mock; reachability still required.
    EXPECT_GT(dist[g.gate_idx], kTestDmax);

    const std::vector<int> path =
        comb::reconstructPath(g.start_idx, g.gate_idx, prev);
    ASSERT_GE(path.size(), 2u);
    EXPECT_TRUE(polylineClear(g, path, map, 0.05));
}

TEST(VoronoiRoadmap, DistanceTransform) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VoronoiGrid grid = comb::buildDistanceTransform(map, 0.05);
    ASSERT_GT(grid.nx, 0);
    ASSERT_GT(grid.ny, 0);
    for (int j = 0; j < grid.ny; ++j) {
        for (int i = 0; i < grid.nx; ++i) {
            const int c = grid.index(i, j);
            if (grid.occupied[c])
                EXPECT_DOUBLE_EQ(grid.dist[c], 0.0);
            else
                EXPECT_GT(grid.dist[c], 0.0);
        }
    }
}

TEST(VoronoiRoadmap, SkeletonClearance) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VoronoiGrid grid = comb::buildDistanceTransform(map, 0.05);
    const std::vector<int> skel =
        comb::extractSkeleton(grid, map.clearance, 20);
    ASSERT_FALSE(skel.empty());
    for (int c : skel) {
        const int i = c % grid.nx;
        const int j = c / grid.nx;
        EXPECT_GE(grid.dist[c], map.clearance);
        EXPECT_FALSE(comb::pointInCollision(grid.world(i, j), map,
                                            map.clearance - grid.res));
    }
}

TEST(VoronoiRoadmap, GraphReachabilityAndClearance) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VoronoiRoadmap vor =
        comb::buildVoronoiRoadmap(map, kStart, victims(), kGate, 0.05, 20, 0.05);
    const comb::VisibilityGraph& g = vor.roadmap;

    std::vector<double> dist;
    std::vector<int> prev;
    comb::dijkstra(g, g.start_idx, dist, prev);
    EXPECT_TRUE(std::isfinite(dist[g.gate_idx]));
    ASSERT_EQ(g.victim_idx.size(), 1u);
    EXPECT_TRUE(std::isfinite(dist[g.victim_idx[0]]));
    EXPECT_LT(dist[g.gate_idx], kTestDmax + 0.50);

    const std::vector<int> path =
        comb::reconstructPath(g.start_idx, g.gate_idx, prev);
    ASSERT_GE(path.size(), 2u);

    const comb::VoronoiGrid& grid = vor.grid;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const comb::Vec2 a = g.nodes[path[i]];
        const comb::Vec2 b = g.nodes[path[i + 1]];
        const double L = comb::dist(a, b);
        const int steps = std::max(1, static_cast<int>(L / 0.05));
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            const comb::Vec2 q{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
            const int gi = static_cast<int>(
                std::lround((q.x - grid.origin_x) / grid.res));
            const int gj = static_cast<int>(
                std::lround((q.y - grid.origin_y) / grid.res));
            if (!grid.inBounds(gi, gj)) continue;
            EXPECT_GE(grid.dist[grid.index(gi, gj)], map.clearance - grid.res);
        }
    }
}

TEST(Orienteering, ZeroVictims) {
    const std::vector<std::vector<double>> D = {{0.0, 3.0}, {3.0, 0.0}};
    const comb::OrienteeringResult r =
        comb::solveOrienteering(D, {}, 100.0, "exact");
    EXPECT_TRUE(r.feasible);
    EXPECT_DOUBLE_EQ(r.total_value, 0.0);
    EXPECT_NEAR(r.total_length, 3.0, 1e-9);
    EXPECT_TRUE(r.victim_order.empty());
}

TEST(Orienteering, SingleVictimBudgetBoundary) {
    const std::vector<std::vector<double>> D = {
        {0.0, 1.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 0.0}};
    const std::vector<double> values = {5.0};

    const comb::OrienteeringResult in =
        comb::solveOrienteering(D, values, 2.0 + 1e-3, "exact");
    EXPECT_TRUE(in.feasible);
    EXPECT_DOUBLE_EQ(in.total_value, 5.0);
    ASSERT_EQ(in.victim_order.size(), 1u);
    EXPECT_EQ(in.victim_order[0], 0);

    const comb::OrienteeringResult out =
        comb::solveOrienteering(D, values, 2.0 - 1e-3, "exact");
    EXPECT_TRUE(out.feasible);
    EXPECT_DOUBLE_EQ(out.total_value, 0.0);
    EXPECT_TRUE(out.victim_order.empty());
}

TEST(Orienteering, TwoVictims) {
    const std::vector<std::vector<double>> D = {
        {0.0, 1.0, 1.0, 1.5},
        {1.0, 0.0, 10.0, 1.0},
        {1.0, 10.0, 0.0, 1.0},
        {1.5, 1.0, 1.0, 0.0}};
    const std::vector<double> values = {3.0, 7.0};

    const comb::OrienteeringResult one =
        comb::solveOrienteering(D, values, 2.5, "exact");
    EXPECT_TRUE(one.feasible);
    EXPECT_DOUBLE_EQ(one.total_value, 7.0);
    ASSERT_EQ(one.victim_order.size(), 1u);
    EXPECT_EQ(one.victim_order[0], 1);

    const comb::OrienteeringResult both =
        comb::solveOrienteering(D, values, 100.0, "exact");
    EXPECT_TRUE(both.feasible);
    EXPECT_DOUBLE_EQ(both.total_value, 10.0);
    EXPECT_EQ(both.victim_order.size(), 2u);

    const comb::OrienteeringResult greedy =
        comb::solveOrienteering(D, values, 100.0, "greedy");
    EXPECT_TRUE(greedy.feasible);
    EXPECT_GT(greedy.total_value, 0.0);
}

TEST(Orienteering, AutoThreshold) {
    auto lineInstance = [](int n) {
        const int P = n + 2;
        std::vector<std::vector<double>> D(P, std::vector<double>(P, 0.0));
        for (int i = 0; i < P; ++i)
            for (int j = 0; j < P; ++j)
                D[i][j] = std::abs(i - j);
        return D;
    };

    {
        const int n = 5;
        const auto D = lineInstance(n);
        const std::vector<double> values(n, 1.0);
        const auto a = comb::solveOrienteering(D, values, 1000.0, "auto");
        const auto e = comb::solveOrienteering(D, values, 1000.0, "exact");
        EXPECT_DOUBLE_EQ(a.total_value, e.total_value);
    }
    {
        const int n = 18;
        const auto D = lineInstance(n);
        const std::vector<double> values(n, 1.0);
        const auto a = comb::solveOrienteering(D, values, 1000.0, "auto");
        const auto g = comb::solveOrienteering(D, values, 1000.0, "greedy");
        EXPECT_TRUE(a.feasible);
        EXPECT_DOUBLE_EQ(a.total_value, g.total_value);
    }
}

TEST(Dubins, LengthLowerBounds) {
    const double kmax = 1.0 / 0.5;
    const comb::DubinsCurve c =
        comb::dubinsShortestPath(0.0, 0.0, 0.0, 3.0, 4.0, 1.0, kmax);
    ASSERT_TRUE(c.valid);
    EXPECT_GE(c.L, std::hypot(3.0, 4.0) - 1e-6);
}

TEST(Dubins, StraightAhead) {
    const double kmax = 1.0 / 0.5;
    const comb::DubinsCurve c =
        comb::dubinsShortestPath(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, kmax);
    ASSERT_TRUE(c.valid);
    EXPECT_NEAR(c.L, 1.0, 1e-6);
}

TEST(Dubins, UTurnHalfCircle) {
    const double r = 0.5;
    const double kmax = 1.0 / r;
    const comb::DubinsCurve c =
        comb::dubinsShortestPath(0.0, 0.0, 0.0, 0.0, 0.6, M_PI, kmax);
    ASSERT_TRUE(c.valid);
    EXPECT_GE(c.L, M_PI * r * 0.9);
}

TEST(Dubins, DiscretizedEndpointAccuracy) {
    const double kmax = 1.0 / 0.5;
    const double v_max = 0.3, dt = 0.01;
    const comb::DubinsCurve c =
        comb::dubinsShortestPath(0.0, 0.0, 0.0, 2.0, 1.0, 0.5, kmax);
    ASSERT_TRUE(c.valid);
    std::vector<comb::RefSample> ref;
    comb::appendDiscretizedDubins(c, v_max, dt, 0.0, ref);
    ASSERT_FALSE(ref.empty());
    const comb::RefSample& last = ref.back();
    EXPECT_NEAR(last.x, c.a3.xf, 0.05);
    EXPECT_NEAR(last.y, c.a3.yf, 0.05);
    double dth = std::fabs(comb::mod2pi(last.theta) - comb::mod2pi(c.a3.thf));
    if (dth > M_PI) dth = 2.0 * M_PI - dth;
    EXPECT_LT(dth, 0.1);
}

namespace {

void checkTourWithinBudget(const comb::TourResult& tour, double dmax,
                           bool require_victim = false) {
    ASSERT_TRUE(tour.feasible);
    ASSERT_FALSE(tour.reference.empty());

    EXPECT_NEAR(tour.reference.front().x, kStart.x, 0.35);
    EXPECT_NEAR(tour.reference.front().y, kStart.y, 0.35);
    EXPECT_NEAR(tour.reference.back().x, kGate.x, 0.5);
    EXPECT_NEAR(tour.reference.back().y, kGate.y, 0.5);

    EXPECT_LE(tour.flyable_length, dmax);
    EXPECT_LE(tour.graph_length, dmax);
    for (const auto& s : tour.reference) EXPECT_NEAR(s.v, kTestVmax, 1e-9);

    if (require_victim) {
        ASSERT_FALSE(tour.victim_order.empty());
        double closest = std::numeric_limits<double>::infinity();
        for (const auto& s : tour.reference)
            closest = std::min(closest, comb::dist({s.x, s.y}, kVictim));
        EXPECT_LE(closest, 0.5);
    }
}

comb::TourResult runPipeline(const comb::Roadmap& roadmap, const comb::GeoMap& map,
                             double dmax) {
    const std::vector<double> values = {10.0};
    return comb::planTour(roadmap, map, /*start_yaw=*/0.0, /*gate_yaw=*/0.0,
                          values, dmax, "auto", kTestVmax, 1.0 / 0.35,
                          /*dt=*/0.01, /*dubins_discretizations=*/16, 0.05);
}
}  // namespace

TEST(Pipeline, VisibilityGateWithin30sBudget) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VisibilityGraph g = comb::buildVisibilityGraph(
        map, kStart, victims(), kGate, map.clearance + 0.10, 0.05);
    checkTourWithinBudget(runPipeline(g, map, kTestDmax), kTestDmax);
}

TEST(Pipeline, VisibilityVisitsVictimWithFixedBudget) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VisibilityGraph g = comb::buildVisibilityGraph(
        map, kStart, victims(), kGate, map.clearance + 0.10, 0.05);
    checkTourWithinBudget(runPipeline(g, map, kVictimBudgetDmax),
                          kVictimBudgetDmax, /*require_victim=*/true);
}

TEST(Pipeline, CellDecompReachesGateWithin30sBudget) {
    const comb::GeoMap map = makeGeoMap();
    const comb::CellDecomposition d =
        comb::buildCellDecomposition(map, kStart, victims(), kGate, 0.05);
    // Raw cell graph exceeds 30 s; planTour budgets simplified line-of-sight path.
    const comb::TourResult tour = runPipeline(d.roadmap, map, kTestDmax);
    checkTourWithinBudget(tour, kTestDmax);
    EXPECT_TRUE(tour.victim_order.empty());
}

TEST(Pipeline, VoronoiReachesGateWithin30sBudget) {
    const comb::GeoMap map = makeGeoMap();
    const comb::VoronoiRoadmap vor =
        comb::buildVoronoiRoadmap(map, kStart, victims(), kGate, 0.05, 20, 0.05);
    // Raw GVD exceeds budget; planTour budgets simplified line-of-sight path.
    const comb::TourResult tour = runPipeline(vor.roadmap, map, kTestDmax);
    checkTourWithinBudget(tour, kTestDmax);
    EXPECT_TRUE(tour.victim_order.empty());
}

TEST(CollisionChecker, IsInsideMap) {
    const WorldModel w = makeWorldModel();
    EXPECT_TRUE(isInsideMap(5.0, 5.0, w.borders));
    EXPECT_FALSE(isInsideMap(11.0, 11.0, w.borders));
}

TEST(CollisionChecker, IsPointValidRaw) {
    const WorldModel w = makeWorldModel();
    EXPECT_TRUE(isPointValid(1.0, 1.0, w));
    EXPECT_TRUE(isPointValid(0.1, 0.1, w));  // raw checker has no clearance
    EXPECT_FALSE(isPointValid(5.0, 5.0, w));
    EXPECT_FALSE(isPointValid(3.0, 3.0, w));
    EXPECT_FALSE(isPointValid(11.0, 11.0, w));
}

TEST(CollisionChecker, IsSegmentValidRaw) {
    const WorldModel w = makeWorldModel();
    EXPECT_TRUE(isSegmentValid(1.0, 1.0, 2.0, 2.0, w));
    EXPECT_FALSE(isSegmentValid(1.0, 1.0, 5.0, 5.0, w));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
