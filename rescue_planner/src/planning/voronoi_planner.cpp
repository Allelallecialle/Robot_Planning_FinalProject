#include "planning/voronoi_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <thread>

#include <std_msgs/Float64MultiArray.h>
#include <loco_planning/Reference.h>

#include "task/tour_builder.hpp"

namespace {
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

VoronoiPlanner::VoronoiPlanner(ros::NodeHandle& nh) {
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
    nh.param("voronoi_res", voronoi_res_, 0.05);
    nh.param("thin_iters", thin_iters_, 20);
    nh.param("dubins_discretizations", dubins_discretizations_, 72);
    nh.param<std::string>("op_method", op_method_, std::string("auto"));

    marker_pub_ =
        nh.advertise<visualization_msgs::Marker>("/voronoi_planner/roadmap", 1);
    ref_pub_ = nh.advertise<loco_planning::Reference>("/" + robot_name_ + "/ref", 10);
    stats_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/voronoi_planner/stats", 1, true);
}

void VoronoiPlanner::initialize(const WorldModel& world) { world_ = &world; }

bool VoronoiPlanner::worldReady() const {
    if (world_ == nullptr) return false;
    return world_->borders.points.size() >= 3 && !world_->gates.empty() &&
           world_->start_ready && world_->timeout_ready &&
           world_->obstacles_ready && world_->victims_ready;
}

void VoronoiPlanner::step() {
    if (done_.load()) return;
    if (!worldReady()) return;
    done_.store(true);
    plan();
    planning_done = true;
}

bool VoronoiPlanner::isPlanningDone() const { return planning_done; }

void VoronoiPlanner::plan() {
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

    // ---------- 2) build the Voronoi (GVD) roadmap ---------------------------
    const double t_road0 = nowMs();
    comb::VoronoiRoadmap vor = comb::buildVoronoiRoadmap(
        map_, start, victims, gate, voronoi_res_, thin_iters_, sample_res_);
    double roadmap_ms = nowMs() - t_road0;

    // Timing safety-valve: retry once at a coarser grid if we blew 30 s.
    if (roadmap_ms > 30000.0) {
        ROS_WARN("[voronoi] roadmap build took %.1f ms (>30 s); retrying coarser.",
                 roadmap_ms);
        const double t_retry = nowMs();
        vor = comb::buildVoronoiRoadmap(map_, start, victims, gate,
                                        voronoi_res_ * 2.0, thin_iters_,
                                        sample_res_);
        roadmap_ms = nowMs() - t_retry;
    }
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        vor_ = vor;
    }
    ROS_INFO("[voronoi] roadmap: %d nodes, %d skeleton cells built in %.2f ms",
             static_cast<int>(vor.roadmap.nodes.size()),
             static_cast<int>(vor.skeleton.size()), roadmap_ms);

    // ---------- 3-5) all-pairs Dijkstra + orienteering + Dubins --------------
    double Dmax = std::numeric_limits<double>::infinity();
    if (world_->victims_timeout > 0)
        Dmax = v_max_ * static_cast<double>(world_->victims_timeout) * dubins_safety_;

    const double t_plan0 = nowMs();
    comb::TourResult tour =
        comb::planTour(vor.roadmap, map_, start_yaw, gate_yaw, values, Dmax,
                       op_method_, v_max_, k_max_, dt_, dubins_discretizations_,
                       sample_res_);
    const double planning_ms = nowMs() - t_plan0;
    total_value_ = tour.total_value;

    ROS_INFO("[voronoi] orienteering: %s, selected %d/%d victims, value=%.1f, "
             "graph length=%.2f m (budget=%.2f m)",
             tour.feasible ? "FEASIBLE" : "INFEASIBLE",
             static_cast<int>(tour.victim_order.size()), n, tour.total_value,
             tour.graph_length, Dmax);

    if (!tour.feasible || tour.reference.empty()) {
        ROS_ERROR("[voronoi] no feasible rescue tour (budget=%.2f m); publishing "
                  "a single stationary waypoint at the start.", Dmax);
        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            waypoints_.clear();
            reference_.clear();
        }
        publishStats(roadmap_ms, planning_ms, 0.0, 0.0, 0);
        loco_planning::Reference msg;
        msg.x_d = start.x; msg.y_d = start.y; msg.theta_d = start_yaw;
        msg.v_d = 0.0; msg.omega_d = 0.0; msg.plan_finished = true;
        ref_pub_.publish(msg);
        return;
    }

    ROS_INFO("[voronoi] planning done in %.2f ms | flyable length=%.2f m | "
             "TOTAL VALUE=%.1f", planning_ms, tour.flyable_length, total_value_);

    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        waypoints_ = tour.waypoints;
        reference_ = tour.reference;
    }
    publishStats(roadmap_ms, planning_ms, total_value_, tour.flyable_length,
                 static_cast<int>(tour.victim_order.size()));
    publishReferenceAsync();
}

void VoronoiPlanner::publishReferenceAsync() {
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

void VoronoiPlanner::publishStats(double roadmap_ms, double planning_ms,
                                  double total_value, double total_length,
                                  int n_selected) const {
    std_msgs::Float64MultiArray msg;
    msg.data = {roadmap_ms, planning_ms, total_value, total_length,
                static_cast<double>(n_selected)};
    stats_pub_.publish(msg);
}

void VoronoiPlanner::visualize() {
    if (!done_.load()) return;

    comb::VoronoiRoadmap vor;
    std::vector<comb::RefSample> ref;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        vor = vor_;
        ref = reference_;
    }
    if (vor.roadmap.nodes.empty()) return;

    // Voronoi skeleton cells (green points).
    visualization_msgs::Marker skel;
    skel.header.frame_id = "map";
    skel.header.stamp = ros::Time::now();
    skel.ns = "voronoi_skeleton";
    skel.id = 0;
    skel.type = visualization_msgs::Marker::POINTS;
    skel.action = visualization_msgs::Marker::ADD;
    skel.scale.x = 0.05; skel.scale.y = 0.05;
    skel.color.a = 0.9; skel.color.g = 1.0;
    for (const auto& p : vor.skeleton_pts) {
        geometry_msgs::Point q; q.x = p.x; q.y = p.y; skel.points.push_back(q);
    }
    marker_pub_.publish(skel);

    // Skeleton graph edges (grey line list).
    visualization_msgs::Marker edges;
    edges.header = skel.header;
    edges.ns = "voronoi_edges";
    edges.id = 1;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.scale.x = 0.015;
    edges.color.a = 0.35; edges.color.r = 0.6; edges.color.g = 0.6; edges.color.b = 0.6;
    const auto& g = vor.roadmap;
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

    // Selected flyable trajectory (red strip).
    visualization_msgs::Marker tr;
    tr.header = skel.header;
    tr.ns = "voronoi_path";
    tr.id = 2;
    tr.type = visualization_msgs::Marker::LINE_STRIP;
    tr.action = visualization_msgs::Marker::ADD;
    tr.scale.x = 0.06;
    tr.color.a = 1.0; tr.color.r = 1.0;
    for (const auto& s : ref) {
        geometry_msgs::Point q; q.x = s.x; q.y = s.y; tr.points.push_back(q);
    }
    marker_pub_.publish(tr);
}
