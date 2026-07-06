#include "planning/visibility_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <std_msgs/Float64MultiArray.h>
#include <loco_planning/Reference.h>

#include "task/orienteering.hpp"
#include "trajectory/dubins_dp.hpp"
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

VisibilityPlanner::VisibilityPlanner(ros::NodeHandle& nh) {
    // --- parameters (defaults mirror loco_nav's planner_base main) ---
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
    nh.param("node_buffer", node_buffer_, 0.0);  // <=0 => clearance + 0.1
    nh.param("dubins_discretizations", dubins_discretizations_, 72);
    nh.param<std::string>("op_method", op_method_, std::string("auto"));

    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visibility_graph", 1);
    ref_pub_ = nh.advertise<loco_planning::Reference>("/" + robot_name_ + "/ref", 10);
    stats_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/visibility_planner/stats", 1, true);
}

void VisibilityPlanner::initialize(const WorldModel& world) {
    world_ = &world;
}

bool VisibilityPlanner::worldReady() const {
    if (world_ == nullptr) return false;
    // We need the geometry, the goal, the robot start and the time budget.
    // Obstacles/victims may legitimately be empty, but we wait until at least
    // their first message arrived so we don't plan on a half-built world.
    return world_->borders.points.size() >= 3 && !world_->gates.empty() &&
           world_->start_ready && world_->timeout_ready &&
           world_->obstacles_ready && world_->victims_ready;
}

void VisibilityPlanner::step() {
    if (done_.load()) return;
    if (!worldReady()) return;
    done_.store(true);  // plan exactly once
    plan();
    planning_done = true;
}

bool VisibilityPlanner::isPlanningDone() const
{
    return planning_done;
}

void VisibilityPlanner::plan() {
    // ---------- 1) build the clearance-aware geometry from the live world ----
    comb::GeoMap map;
    map.clearance = robot_radius_ + safety_margin_;
    for (const auto& p : world_->borders.points)
        map.border.push_back({p.x, p.y});
    for (const auto& obs : world_->obstacles) {
        comb::Obstacle o;
        // Convention from send_obstacles.cpp: cylinder => 1 polygon point +
        // radius>0; box/polygon => vertices and radius 0.
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
        values.push_back(v.value);  // value = weight (smuggled in radius field)
    }
    const int n = static_cast<int>(victims.size());

    const double node_buffer =
        (node_buffer_ > 0.0) ? node_buffer_ : (map.clearance + 0.10);

    // ---------- 2) build the visibility graph (roadmap) ----------------------
    const double t_road0 = nowMs();
    graph_ = comb::buildVisibilityGraph(map_, start, victims, gate, node_buffer,
                                        sample_res_);
    const double roadmap_ms = nowMs() - t_road0;
    ROS_INFO("[visibility] roadmap: %d nodes built in %.2f ms",
             static_cast<int>(graph_.nodes.size()), roadmap_ms);

    if (metrics_) {
        metrics_->roadmap_nodes = static_cast<int>(graph_.nodes.size());
        int edges = 0;
        for (int i = 0; i < static_cast<int>(graph_.adj.size()); ++i)
            for (int j : graph_.adj[i])
                if (j > i) ++edges;
        metrics_->roadmap_edges = edges;
    }

    // ---------- 3) all-pairs shortest distances between POIs ------------------
    const double t_plan0 = nowMs();
    std::vector<int> poi_node;  // graph index of each POI
    poi_node.push_back(graph_.start_idx);
    for (int idx : graph_.victim_idx) poi_node.push_back(idx);
    poi_node.push_back(graph_.gate_idx);
    const int P = static_cast<int>(poi_node.size());  // == n + 2

    std::vector<std::vector<double>> D(P, std::vector<double>(P, 0.0));
    std::vector<std::vector<int>> prevs(P);  // dijkstra predecessor per POI
    for (int p = 0; p < P; ++p) {
        std::vector<double> dist;
        comb::dijkstra(graph_, poi_node[p], dist, prevs[p]);
        for (int q = 0; q < P; ++q) D[p][q] = dist[poi_node[q]];
    }

    // ---------- 4) orienteering: pick victim subset & order ------------------
    // Budget: distance the robot can travel within the timeout at v_max,
    // derated by `dubins_safety` to leave room for curvature overhead (the
    // flyable Dubins path is always a bit longer than the straight graph path).
    const double Dmax =
        comb::distanceBudget(world_->victims_timeout, v_max_, dubins_safety_);

    comb::OrienteeringResult op =
        comb::solveOrienteering(D, values, Dmax, op_method_);
    total_value_ = op.total_value;
    ROS_INFO("[visibility] orienteering: %s, selected %d/%d victims, value=%.1f, "
             "graph length=%.2f m (budget=%.2f m)",
             op.feasible ? "FEASIBLE" : "INFEASIBLE",
             static_cast<int>(op.victim_order.size()), n, op.total_value,
             op.total_length, Dmax);
    if (!op.feasible) {
        ROS_WARN("[visibility] cannot reach the gate within budget; aborting.");
        const double planning_ms = nowMs() - t_plan0;
        if (metrics_) {
            metrics_->victims = 0;
            metrics_->path_length = 0.0;
            metrics_->score = 0;
            metrics_->success = false;
        }
        publishStats(roadmap_ms, planning_ms, 0.0, 0.0, 0);
        return;
    }

    // ---------- 5) expand POI order into a geometric waypoint polyline -------
    std::vector<int> poi_order;
    poi_order.push_back(0);                       // start
    for (int vi : op.victim_order) poi_order.push_back(vi + 1);
    poi_order.push_back(P - 1);                   // gate

    tour_nodes_.clear();
    for (size_t k = 0; k + 1 < poi_order.size(); ++k) {
        const int a = poi_order[k];
        const int b = poi_order[k + 1];
        std::vector<int> seg =
            comb::reconstructPath(poi_node[a], poi_node[b], prevs[a]);
        if (seg.empty()) {
            ROS_WARN("[visibility] missing graph path between POIs %d-%d", a, b);
            continue;
        }
        for (size_t t = (tour_nodes_.empty() ? 0 : 1); t < seg.size(); ++t)
            tour_nodes_.push_back(seg[t]);
    }

    std::vector<comb::Vec2> waypoints;
    for (int idx : tour_nodes_) {
        const comb::Vec2 p = graph_.nodes[idx];
        if (waypoints.empty() || comb::dist(waypoints.back(), p) > 1e-6)
            waypoints.push_back(p);
    }
    if (waypoints.size() < 2) {
        ROS_WARN("[visibility] degenerate waypoint list; aborting.");
        return;
    }

    // ---------- 6) heading DP + Dubins stitching with clearance re-check -----
    // Intermediate (victim) headings are free; start/gate headings are fixed.
    // After stitching, each curved arc is re-sampled and verified against the
    // ORIGINAL obstacles (real clearance), because the Dubins arcs can bulge
    // outside the straight visibility corridor. If a leg clips, we insert the
    // midpoint of the offending straight segment (guaranteed clear, since it
    // came from the visibility graph) and re-optimise -- this shrinks the arc.
    std::vector<comb::Vec2> pts = waypoints;
    std::vector<comb::RefSample> ref;
    const int max_subdiv = 12;
    for (int iter = 0; iter <= max_subdiv; ++iter) {
        const std::vector<double> ang = comb::optimizeHeadings(
            pts, start_yaw, gate_yaw, k_max_, dubins_discretizations_);

        ref.clear();
        double t_off = 0.0;
        int bad_leg = -1;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const comb::DubinsCurve c = comb::dubinsShortestPath(
                pts[i].x, pts[i].y, ang[i], pts[i + 1].x, pts[i + 1].y,
                ang[i + 1], k_max_);
            if (!c.valid) { bad_leg = static_cast<int>(i); break; }

            std::vector<comb::RefSample> leg;
            comb::appendDiscretizedDubins(c, v_max_, dt_, t_off, leg);

            bool clear = true;
            for (const auto& s : leg) {
                if (comb::pointInCollision({s.x, s.y}, map_)) { clear = false; break; }
            }
            if (!clear) { bad_leg = static_cast<int>(i); break; }

            // Append (skip first sample of non-initial legs to avoid duplicates).
            for (size_t s = (i == 0 ? 0 : 1); s < leg.size(); ++s)
                ref.push_back(leg[s]);
            t_off = ref.empty() ? 0.0 : ref.back().t + dt_;
        }

        if (bad_leg < 0) break;  // fully clear
        const comb::Vec2 mid{(pts[bad_leg].x + pts[bad_leg + 1].x) / 2.0,
                             (pts[bad_leg].y + pts[bad_leg + 1].y) / 2.0};
        pts.insert(pts.begin() + bad_leg + 1, mid);
        if (iter == max_subdiv)
            ROS_WARN("[visibility] residual arc collision after %d subdivisions",
                     max_subdiv);
    }

    const double traj_len = v_max_ * (ref.empty() ? 0.0 : ref.back().t);
    const double planning_ms = nowMs() - t_plan0;
    ROS_INFO("[visibility] planning done in %.2f ms | flyable length=%.2f m, "
             "duration=%.1f s | TOTAL VALUE=%.1f",
             planning_ms, traj_len, ref.empty() ? 0.0 : ref.back().t,
             total_value_);
    if (world_->victims_timeout > 0 &&
        (!ref.empty() && ref.back().t > world_->victims_timeout))
        ROS_WARN("[visibility] flyable duration exceeds timeout (%.1f > %d s)!",
                 ref.back().t, world_->victims_timeout);

    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        waypoints_ = waypoints;
        reference_ = ref;
    }
    if (metrics_) {
        metrics_->victims = static_cast<int>(op.victim_order.size());
        metrics_->path_length = traj_len;
        metrics_->score = op.total_value;
        metrics_->success = true;
    }
    publishStats(roadmap_ms, planning_ms, total_value_, traj_len,
                 static_cast<int>(op.victim_order.size()));

    publishReferenceAsync();
}

void VisibilityPlanner::publishReferenceAsync() {
    std::vector<comb::RefSample> ref;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        ref = reference_;
    }
    if (ref.empty()) return;

    ros::Publisher pub = ref_pub_;
    const double dt = dt_;

    // Stream the reference in (sim) real time, exactly like planner_base.py:
    // one sample per dt, then a final message flagged plan_finished=true.
    std::thread([pub, ref, dt]() mutable {
        ros::Rate rate(1.0 / dt);
        for (const auto& s : ref) {
            if (!ros::ok()) return;
            loco_planning::Reference msg;
            msg.x_d = s.x;
            msg.y_d = s.y;
            msg.theta_d = s.theta;
            msg.v_d = s.v;
            msg.omega_d = s.omega;
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
        ROS_INFO("[visibility] reference fully published.");
    }).detach();
}

void VisibilityPlanner::publishStats(double roadmap_ms, double planning_ms,
                                     double total_value, double total_length,
                                     int n_selected) const {
    std_msgs::Float64MultiArray msg;
    msg.data = {roadmap_ms, planning_ms, total_value, total_length,
                static_cast<double>(n_selected)};
    stats_pub_.publish(msg);
}

void VisibilityPlanner::visualize() {
    if (!done_.load()) return;

    std::vector<comb::Vec2> nodes;
    std::vector<std::pair<int, int>> edges;
    std::vector<comb::RefSample> ref;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        nodes = graph_.nodes;
        for (int i = 0; i < static_cast<int>(graph_.adj.size()); ++i)
            for (int j : graph_.adj[i])
                if (j > i) edges.emplace_back(i, j);
        ref = reference_;
    }
    if (nodes.empty()) return;

    // graph edges (faint grey)
    visualization_msgs::Marker e;
    e.header.frame_id = "map";
    e.header.stamp = ros::Time::now();
    e.ns = "visibility_edges";
    e.id = 0;
    e.type = visualization_msgs::Marker::LINE_LIST;
    e.action = visualization_msgs::Marker::ADD;
    e.scale.x = 0.02;
    e.color.a = 0.35; e.color.r = 0.6; e.color.g = 0.6; e.color.b = 0.6;
    for (const auto& ed : edges) {
        geometry_msgs::Point a, b;
        a.x = nodes[ed.first].x;  a.y = nodes[ed.first].y;
        b.x = nodes[ed.second].x; b.y = nodes[ed.second].y;
        e.points.push_back(a);
        e.points.push_back(b);
    }
    marker_pub_.publish(e);

    // graph nodes (blue)
    visualization_msgs::Marker nd;
    nd.header = e.header;
    nd.ns = "visibility_nodes";
    nd.id = 1;
    nd.type = visualization_msgs::Marker::POINTS;
    nd.action = visualization_msgs::Marker::ADD;
    nd.scale.x = 0.10; nd.scale.y = 0.10;
    nd.color.a = 1.0; nd.color.b = 1.0;
    for (const auto& p : nodes) {
        geometry_msgs::Point q; q.x = p.x; q.y = p.y; nd.points.push_back(q);
    }
    marker_pub_.publish(nd);

    // final flyable trajectory (red line)
    visualization_msgs::Marker tr;
    tr.header = e.header;
    tr.ns = "visibility_path";
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
