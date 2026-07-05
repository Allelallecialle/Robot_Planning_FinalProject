#pragma once

// ============================================================================
//  voronoi_planner.hpp  --  COMBINATORIAL planner (max-clearance / GVD roadmap)
// ----------------------------------------------------------------------------
//  Sibling of VisibilityPlanner / CellDecompPlanner: same `Planner` interface
//  (initialize/step/visualize), selected via `planner_type:=voronoi`.
//
//  Pipeline (computed ONCE at startup), identical to the other combinatorial
//  planners except for stage 2:
//    1. build the clearance-aware geometry from the live world      (geometry)
//    2. build the Generalised Voronoi Diagram roadmap               (roadmap)
//    3. Dijkstra all-pairs between {start, victims, gate}           (collapse)
//    4. orienteering: pick victim subset/order within the budget    (selection)
//    5. heading DP + Dubins stitching with clearance re-check       (path)
//    6. publish the trajectory as loco_planning/Reference           (stream)
//
//  Algorithm properties (see voronoi_roadmap.hpp):
//    * Maximum-clearance paths (maximum safety), generally LONGER than the
//      visibility-graph shortest path (safety/length trade-off).
//    * Grid-discrete approximation of the true GVD; NOT exact-complete: no path
//      is returned if the discrete skeleton does not connect two POIs.
// ============================================================================

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "planner.hpp"
#include "world_model.hpp"
#include "task/voronoi_roadmap.hpp"
#include "trajectory/dubins.hpp"

class VoronoiPlanner : public Planner {
public:
    explicit VoronoiPlanner(ros::NodeHandle& nh);

    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;
    bool isPlanningDone() const override;

    bool planning_done = false;

private:
    // --- pipeline ---
    bool worldReady() const;
    void plan();
    void publishReferenceAsync();
    void publishStats(double roadmap_ms, double planning_ms, double total_value,
                      double total_length, int n_selected) const;

    // --- parameters (ROS-configurable) ---
    std::string robot_name_;
    double robot_radius_;
    double safety_margin_;
    double v_max_;
    double k_max_;
    double victim_radius_;
    double dt_;
    double dubins_safety_;
    double sample_res_;
    double voronoi_res_;               // GVD grid resolution
    int    thin_iters_;                // morphological thinning iterations
    int    dubins_discretizations_;
    std::string op_method_;

    // --- ROS I/O ---
    ros::Publisher marker_pub_;
    ros::Publisher ref_pub_;
    ros::Publisher stats_pub_;

    // --- state ---
    const WorldModel* world_ = nullptr;
    std::atomic<bool> done_{false};

    comb::GeoMap             map_;
    comb::VoronoiRoadmap     vor_;
    std::vector<comb::Vec2>  waypoints_;
    std::vector<comb::RefSample> reference_;
    std::mutex               data_mtx_;
    double                   total_value_ = 0.0;
};
