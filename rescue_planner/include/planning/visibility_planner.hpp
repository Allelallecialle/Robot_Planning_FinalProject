#pragma once

// ============================================================================
//  visibility_planner.hpp  --  COMBINATORIAL "Target Rescue" planner
// ----------------------------------------------------------------------------
//  Sibling of the teammate's sampling-based planners (PRM/RRT/RRT*): it
//  implements the very same `Planner` interface (initialize/step/visualize)
//  and is selected via `planner_type:=visibility`.
//
//  Strategy (all computed ONCE at startup):
//    1. inflate obstacles/border by the robot clearance              (geometry)
//    2. build an exact visibility graph over inflated corners + POIs (roadmap)
//    3. Dijkstra all-pairs distances between {start, victims, gate}   (collapse)
//    4. orienteering: pick the victim subset/order maximising value   (selection)
//       within the time budget (/victims_timeout * v_max)
//    5. choose victim headings (DP) and stitch Dubins manoeuvres,     (path)
//       re-checking real clearance on the curved arcs
//    6. publish the trajectory as loco_planning/Reference on /<robot>/ref
//       (same convention as loco_nav's planner_base.py)
//
//  Timing of roadmap construction, planning and the selected total value are
//  logged/published separately because the evaluation rewards each individually.
// ============================================================================

#include <atomic>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "planner.hpp"
#include "world_model.hpp"
#include "utils/geometry_utils.hpp"
#include "task/visibility_graph.hpp"
#include "trajectory/dubins.hpp"

class VisibilityPlanner : public Planner {
public:
    explicit VisibilityPlanner(ros::NodeHandle& nh);

    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;
    bool isPlanningDone() const override;


    bool planning_done = false;

private:
    // --- pipeline ---
    bool worldReady() const;
    void plan();                       // runs the whole pipeline once
    void publishReferenceAsync();      // streams the reference on /<robot>/ref
    void publishStats(double roadmap_ms, double planning_ms,
                      double total_value, double total_length,
                      int n_selected) const;

    // --- parameters (ROS-configurable) ---
    std::string robot_name_;
    double robot_radius_;
    double safety_margin_;
    double v_max_;
    double k_max_;                     // max curvature = 1/turning_radius
    double victim_radius_;             // geometric victim disk radius (0.5 m)
    double dt_;
    double dubins_safety_;             // derating of the distance budget
    double sample_res_;                // collision sampling resolution
    double node_buffer_;               // corner-waypoint inflation
    int    dubins_discretizations_;
    std::string op_method_;            // "auto" | "exact" | "greedy"

    // --- ROS I/O ---
    ros::Publisher marker_pub_;
    ros::Publisher ref_pub_;
    ros::Publisher stats_pub_;

    // --- state ---
    const WorldModel* world_ = nullptr;
    std::atomic<bool> done_{false};

    // results kept for visualization / publishing
    comb::GeoMap          map_;
    comb::VisibilityGraph graph_;
    std::vector<int>      tour_nodes_;              // graph node indices of tour
    std::vector<comb::Vec2> waypoints_;             // POI-expanded polyline
    std::vector<comb::RefSample> reference_;        // final discretized path
    std::mutex            data_mtx_;
    double                total_value_ = 0.0;
};
