#include "planning/cell_decomp_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <thread>

#include <std_msgs/Float64MultiArray.h>
#include <loco_planning/Reference.h>

#include "task/tour_builder.hpp"
#include "utils/planning_budget.hpp"

namespace {
// yaw from a ROS quaternion (z-axis rotation), same formula as planner_base.py.
double yawFromQuat(const geometry_msgs::Quaternion& q) {
    const double t0 = 2.0 * (q.w * q.z + q.x * q.y);
    const double t1 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(t0, t1);
}
double nowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

CellDecompPlanner::CellDecompPlanner(ros::NodeHandle& nh)
    : CellDecompPlanner(nh, false) {}

CellDecompPlanner::CellDecompPlanner(ros::NodeHandle& nh, bool approx)
    : approx_(approx) {
    nh.param<std::string>("robot_name", robot_name_, std::string("limo0"));
    nh.param("robot_radius", robot_radius_, 0.20);
    nh.param("safety_margin", safety_margin_, 0.10);
    nh.param("v_max", v_max_, 0.30);
    double turning_radius;
    nh.param("turning_radius", turning_radius, 0.35);
    k_max_ = 1.0 / turning_radius;
    nh.param("victim_radius", victim_radius_, 0.50);
    nh.param("dt", dt_, 0.01);
    nh.param("dubins_safety", dubins_safety_, 0.85);
    nh.param("sample_res", sample_res_, 0.05);
    nh.param("cell_size", cell_size_, 0.10);
    nh.param("dubins_discretizations", dubins_discretizations_, 72);
    nh.param<std::string>("op_method", op_method_, std::string("auto"));

    marker_pub_ =
        nh.advertise<visualization_msgs::Marker>("/cell_decomp_planner/roadmap", 1);
    ref_pub_ = nh.advertise<loco_planning::Reference>("/" + robot_name_ + "/ref", 10);
    stats_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/cell_decomp_planner/stats", 1, true);
}

void CellDecompPlanner::initialize(const WorldModel& world) {
    world_ = &world;
}

bool CellDecompPlanner::worldReady() const {
    if (world_ == nullptr) return false;
    return world_->borders.points.size() >= 3 && !world_->gates.empty() &&
           world_->start_ready && world_->timeout_ready &&
           world_->obstacles_ready && world_->victims_ready;
}

void CellDecompPlanner::step() {
    if (done_.load()) return;
    if (!worldReady()) return;
    done_.store(true);  // plan exactly once
    plan();
    planning_done = true;
}

bool CellDecompPlanner::isPlanningDone() const { return planning_done; }

void CellDecompPlanner::plan() {
    const char* tag = approx_ ? "cell_decomp_approx" : "cell_decomp";

    // ---------- 1) clearance-aware geometry from the live world --------------
    comb::GeoMap map;
    map.clearance = robot_radius_ + safety_margin_;
    for (const auto& p : world_->borders.points)
        map.border.push_back({p.x, p.y});
    for (const auto& obs : world_->obstacles) {
        comb::Obstacle o;
        if (obs.radius > 0.0 && obs.polygon.points.size() <= 1) {
            o.is_circle = true;
            o.center = {obs.polygon.points.empty() ? 0.0 : obs.polygon.points[0].x,
                        obs.polygon.points.empty() ? 0.0 : obs.polygon.points[0].y};
            o.radius = obs.radius;
        } else {
            o.is_circle = false;
            for (const auto& p : obs.polygon.points) o.verts.push_back({p.x, p.y});
        }
        map.obstacles.push_back(o);
    }
    map_ = map;

    const comb::Vec2 start{world_->start.x, world_->start.y};
    const double start_yaw = world_->start.yaw;
    const comb::Vec2 gate{world_->gates[0].position.x, world_->gates[0].position.y};
    const double gate_yaw = yawFromQuat(world_->gates[0].orientation);

    std::vector<comb::Vec2> victims;
    std::vector<double> values;
    for (const auto& v : world_->victims) {
        victims.push_back({v.x, v.y});
        values.push_back(v.value);
    }
    const int n = static_cast<int>(victims.size());

    // ---------- 2) build the cell-decomposition roadmap ----------------------
    const double t_road0 = nowMs();
    comb::CellDecomposition decomp =
        approx_ ? comb::buildApproxCellDecomposition(map_, start, victims, gate,
                                                     cell_size_, sample_res_)
                : comb::buildCellDecomposition(map_, start, victims, gate,
                                               sample_res_);
    double roadmap_ms = nowMs() - t_road0;

    // Timing safety-valve: fall back to a coarser resolution once if the build
    // blew the 30 s budget with default parameters.
    if (roadmap_ms > 30000.0) {
        ROS_WARN("[%s] roadmap build took %.1f ms (>30 s); retrying coarser.",
                 tag, roadmap_ms);
        const double t_retry = nowMs();
        if (approx_)
            decomp = comb::buildApproxCellDecomposition(
                map_, start, victims, gate, cell_size_ * 2.0, sample_res_ * 2.0);
        else
            decomp = comb::buildCellDecomposition(map_, start, victims, gate,
                                                  sample_res_ * 2.0);
        roadmap_ms = nowMs() - t_retry;
    }
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        decomp_ = decomp;
    }
    ROS_INFO("[%s] roadmap: %d nodes, %d cells built in %.2f ms", tag,
             static_cast<int>(decomp.roadmap.nodes.size()),
             static_cast<int>(decomp.cells.size()), roadmap_ms);

    if (metrics_) {
        metrics_->roadmap_nodes = static_cast<int>(decomp.roadmap.nodes.size());
        int edges = 0;
        for (int i = 0; i < static_cast<int>(decomp.roadmap.adj.size()); ++i)
            for (int j : decomp.roadmap.adj[i])
                if (j > i) ++edges;
        metrics_->roadmap_edges = edges;
    }

    // ---------- 3-5) all-pairs Dijkstra + orienteering + Dubins --------------
    // The orienteering budget bounds the STRAIGHT graph length, but the robot
    // flies longer Dubins arcs. The real limit is that the FLYABLE trajectory
    // must be coverable within the timeout: flyable_length <= v_max * timeout.
    double Dmax =
        comb::distanceBudget(world_->victims_timeout, v_max_, dubins_safety_);
    const double flyable_budget =
        (world_->victims_timeout > 0)
            ? v_max_ * static_cast<double>(world_->victims_timeout)
            : std::numeric_limits<double>::infinity();

    const double t_plan0 = nowMs();
    comb::TourResult tour =
        comb::planTour(decomp.roadmap, map_, start_yaw, gate_yaw, values, Dmax,
                       op_method_, v_max_, k_max_, dt_, dubins_discretizations_,
                       sample_res_);

    // Feedback: if the selected tour's flyable trajectory would overrun the
    // timeout, tighten the graph budget in proportion to the observed overrun
    // and re-plan, so the executed lap respects the timeout instead of it.
    for (int budget_iter = 0;
         budget_iter < 8 && std::isfinite(flyable_budget) && tour.feasible &&
         !tour.reference.empty() && tour.flyable_length > flyable_budget;
         ++budget_iter) {
        Dmax *= (flyable_budget / tour.flyable_length) * 0.98;
        ROS_INFO("[%s] flyable %.2f m > budget %.2f m; retrying with Dmax=%.2f m",
                 tag, tour.flyable_length, flyable_budget, Dmax);
        tour = comb::planTour(decomp.roadmap, map_, start_yaw, gate_yaw, values,
                              Dmax, op_method_, v_max_, k_max_, dt_,
                              dubins_discretizations_, sample_res_);
    }
    const double planning_ms = nowMs() - t_plan0;
    total_value_ = tour.total_value;

    ROS_INFO("[%s] orienteering: %s, selected %d/%d victims, value=%.1f, "
             "graph length=%.2f m (budget=%.2f m)",
             tag, tour.feasible ? "FEASIBLE" : "INFEASIBLE",
             static_cast<int>(tour.victim_order.size()), n, tour.total_value,
             tour.graph_length, Dmax);

    if (!tour.feasible || tour.reference.empty()) {
        ROS_ERROR("[%s] no feasible rescue tour (budget=%.2f m); publishing a "
                  "single stationary waypoint at the start.", tag, Dmax);
        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            waypoints_.clear();
            reference_.clear();
        }
        if (metrics_) {
            metrics_->victims = 0;
            metrics_->path_length = 0.0;
            metrics_->score = 0;
            metrics_->success = false;
        }
        publishStats(roadmap_ms, planning_ms, 0.0, 0.0, 0);
        // Publish one finished reference at the start so the controller stops.
        loco_planning::Reference msg;
        msg.x_d = start.x; msg.y_d = start.y; msg.theta_d = start_yaw;
        msg.v_d = 0.0; msg.omega_d = 0.0; msg.plan_finished = true;
        ref_pub_.publish(msg);
        return;
    }

    ROS_INFO("[%s] planning done in %.2f ms | flyable length=%.2f m | "
             "TOTAL VALUE=%.1f", tag, planning_ms, tour.flyable_length,
             total_value_);

    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        waypoints_ = tour.waypoints;
        reference_ = tour.reference;
    }
    if (metrics_) {
        metrics_->victims = static_cast<int>(tour.victim_order.size());
        metrics_->path_length = tour.flyable_length;
        metrics_->score = tour.total_value;
        metrics_->success = true;
    }
    publishStats(roadmap_ms, planning_ms, total_value_, tour.flyable_length,
                 static_cast<int>(tour.victim_order.size()));
    publishReferenceAsync();
}

void CellDecompPlanner::publishReferenceAsync() {
    std::vector<comb::RefSample> ref;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        ref = reference_;
    }
    if (ref.empty()) return;

    ros::Publisher pub = ref_pub_;
    const double dt = dt_;
    std::thread([pub, ref, dt]() mutable {
        ros::Rate rate(1.0 / dt);
        for (const auto& s : ref) {
            if (!ros::ok()) return;
            loco_planning::Reference msg;
            msg.x_d = s.x; msg.y_d = s.y; msg.theta_d = s.theta;
            msg.v_d = s.v; msg.omega_d = s.omega;
            msg.plan_finished = false;
            pub.publish(msg);
            rate.sleep();
        }
        loco_planning::Reference last;
        const auto& s = ref.back();
        last.x_d = s.x; last.y_d = s.y; last.theta_d = s.theta;
        last.v_d = 0.0; last.omega_d = 0.0;
        last.plan_finished = true;
        pub.publish(last);
    }).detach();
}

void CellDecompPlanner::publishStats(double roadmap_ms, double planning_ms,
                                     double total_value, double total_length,
                                     int n_selected) const {
    std_msgs::Float64MultiArray msg;
    msg.data = {roadmap_ms, planning_ms, total_value, total_length,
                static_cast<double>(n_selected)};
    stats_pub_.publish(msg);
}

void CellDecompPlanner::visualize() {
    if (!done_.load()) return;

    comb::CellDecomposition decomp;
    std::vector<comb::RefSample> ref;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        decomp = decomp_;
        ref = reference_;
    }
    if (decomp.roadmap.nodes.empty()) return;

    visualization_msgs::Marker cells;
    cells.header.frame_id = "map";
    cells.header.stamp = ros::Time::now();
    cells.ns = "cell_decomp_cells";
    cells.id = 0;
    cells.type = visualization_msgs::Marker::CUBE_LIST;
    cells.action = visualization_msgs::Marker::ADD;
    cells.scale.x = 0.12; cells.scale.y = 0.12; cells.scale.z = 0.02;
    cells.color.a = 0.35; cells.color.g = 0.6; cells.color.b = 0.9;
    for (const auto& c : decomp.cells) {
        geometry_msgs::Point q;
        q.x = 0.5 * (c.x_lo + c.x_hi); q.y = 0.5 * (c.y_lo + c.y_hi);
        cells.points.push_back(q);
    }
    marker_pub_.publish(cells);

    // Cell-adjacency edges (grey line list), skipping POI stubs for clarity.
    visualization_msgs::Marker edges;
    edges.header = cells.header;
    edges.ns = "cell_decomp_edges";
    edges.id = 1;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.scale.x = 0.02;
    edges.color.a = 0.35; edges.color.r = 0.6; edges.color.g = 0.6; edges.color.b = 0.6;
    const auto& g = decomp.roadmap;
    for (int i = 0; i < static_cast<int>(g.adj.size()); ++i) {
        for (int j : g.adj[i]) {
            if (j <= i) continue;
            geometry_msgs::Point a, b;
            a.x = g.nodes[i].x; a.y = g.nodes[i].y;
            b.x = g.nodes[j].x; b.y = g.nodes[j].y;
            edges.points.push_back(a);
            edges.points.push_back(b);
        }
    }
    marker_pub_.publish(edges);

    // Points of interest (yellow spheres).
    visualization_msgs::Marker pois;
    pois.header = cells.header;
    pois.ns = "cell_decomp_pois";
    pois.id = 2;
    pois.type = visualization_msgs::Marker::SPHERE_LIST;
    pois.action = visualization_msgs::Marker::ADD;
    pois.scale.x = 0.20; pois.scale.y = 0.20; pois.scale.z = 0.20;
    pois.color.a = 1.0; pois.color.r = 1.0; pois.color.g = 1.0;
    auto pushPoi = [&](int idx) {
        if (idx < 0) return;
        geometry_msgs::Point q; q.x = g.nodes[idx].x; q.y = g.nodes[idx].y;
        pois.points.push_back(q);
    };
    pushPoi(g.start_idx);
    for (int vi : g.victim_idx) pushPoi(vi);
    pushPoi(g.gate_idx);
    marker_pub_.publish(pois);

    // Selected flyable trajectory (red arrow trail approximated by a strip).
    visualization_msgs::Marker tr;
    tr.header = cells.header;
    tr.ns = "cell_decomp_path";
    tr.id = 3;
    tr.type = visualization_msgs::Marker::LINE_STRIP;
    tr.action = visualization_msgs::Marker::ADD;
    tr.scale.x = 0.06;
    tr.color.a = 1.0; tr.color.r = 1.0;
    for (const auto& s : ref) {
        geometry_msgs::Point q; q.x = s.x; q.y = s.y; tr.points.push_back(q);
    }
    marker_pub_.publish(tr);
}
