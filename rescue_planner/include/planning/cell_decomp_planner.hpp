#pragma once

// ============================================================================
//  cell_decomp_planner.hpp  --  COMBINATORIAL planner (cell decomposition)
// ----------------------------------------------------------------------------
//  Sibling of VisibilityPlanner and of the teammate's sampling planners: it
//  implements the same `Planner` interface (initialize/step/visualize) and is
//  selected via `planner_type:=cell_decomp` (exact vertical/trapezoidal
//  decomposition) or `planner_type:=cell_decomp_approx` (grid decomposition).
//
//  Pipeline (computed ONCE at startup), identical to VisibilityPlanner except
//  for stage 2:
//    1. build the clearance-aware geometry from the live world      (geometry)
//    2. build the cell-decomposition roadmap                        (roadmap)
//    3. Dijkstra all-pairs between {start, victims, gate}           (collapse)
//    4. orienteering: pick victim subset/order within the budget    (selection)
//    5. heading DP + Dubins stitching with clearance re-check       (path)
//    6. publish the trajectory as loco_planning/Reference           (stream)
//
//  Algorithm properties (see cell_decomposition.hpp):
//    * exact  : COMPLETE for cell-adjacency paths; moderate path quality
//               (constrained to cell reps); zero-clearance NOT guaranteed but
//               all reps keep the robot clearance from obstacles.
//    * approx : NOT complete; quality depends on the grid resolution.
//  Known limitation: the roadmap is graph-restricted, so paths are generally
//  longer than the visibility-graph shortest path.
// ============================================================================

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "planner.hpp"
#include "world_model.hpp"
#include "task/cell_decomposition.hpp"
#include "trajectory/dubins.hpp"

class CellDecompPlanner : public Planner {
public:
    explicit CellDecompPlanner(ros::NodeHandle& nh);

    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;
    bool isPlanningDone() const override;

    bool planning_done = false;

protected:
    // `approx == true` selects the grid decomposition variant. Derived classes
    // (CellDecompApproxPlanner) use this to switch the roadmap builder and the
    // published topic names.
    CellDecompPlanner(ros::NodeHandle& nh, bool approx);

private:
    // --- pipeline ---
    bool worldReady() const;
    void plan();
    void publishReferenceAsync();
    void publishStats(double roadmap_ms, double planning_ms, double total_value,
                      double total_length, int n_selected) const;

    // --- parameters (ROS-configurable) ---
    bool        approx_;
    std::string robot_name_;
    double robot_radius_;
    double safety_margin_;
    double v_max_;
    double k_max_;                     // max curvature = 1/turning_radius
    double victim_radius_;
    double dt_;
    double dubins_safety_;
    double sample_res_;
    double cell_size_;                 // grid cell side (approx variant)
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
    comb::CellDecomposition  decomp_;
    std::vector<comb::Vec2>  waypoints_;
    std::vector<comb::RefSample> reference_;
    std::mutex               data_mtx_;
    double                   total_value_ = 0.0;
};

/**
 * @brief Approximate (grid) cell-decomposition planner (planner_type:=cell_decomp_approx).
 *
 * Thin specialisation of CellDecompPlanner that selects the grid decomposition
 * roadmap. NOT complete; provided as the approximate counterpart of the exact
 * trapezoidal decomposition for the report's comparison.
 */
class CellDecompApproxPlanner : public CellDecompPlanner {
public:
    explicit CellDecompApproxPlanner(ros::NodeHandle& nh)
        : CellDecompPlanner(nh, true) {}
};
