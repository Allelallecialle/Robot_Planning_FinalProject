#pragma once

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
    CellDecompPlanner(ros::NodeHandle& nh, bool approx);

private:
    bool worldReady() const;
    void plan();
    void publishReferenceAsync();
    void publishStats(double roadmap_ms, double planning_ms, double total_value,
                      double total_length, int n_selected) const;

    bool        approx_;
    std::string robot_name_;
    double robot_radius_;
    double safety_margin_;
    double v_max_;
    double k_max_;
    double victim_radius_;
    double dt_;
    double dubins_safety_;
    double sample_res_;
    double cell_size_;
    int    dubins_discretizations_;
    std::string op_method_;

    ros::Publisher marker_pub_;
    ros::Publisher ref_pub_;
    ros::Publisher stats_pub_;

    const WorldModel* world_ = nullptr;
    std::atomic<bool> done_{false};

    comb::GeoMap             map_;
    comb::CellDecomposition  decomp_;
    std::vector<comb::Vec2>  waypoints_;
    std::vector<comb::RefSample> reference_;
    std::mutex               data_mtx_;
    double                   total_value_ = 0.0;
};

class CellDecompApproxPlanner : public CellDecompPlanner {
public:
    explicit CellDecompApproxPlanner(ros::NodeHandle& nh)
        : CellDecompPlanner(nh, true) {}
};
