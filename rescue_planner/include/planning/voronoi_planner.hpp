#pragma once

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
    bool worldReady() const;
    void plan();
    void publishReferenceAsync();
    void publishStats(double roadmap_ms, double planning_ms, double total_value,
                      double total_length, int n_selected) const;

    std::string robot_name_;
    double robot_radius_;
    double safety_margin_;
    double v_max_;
    double k_max_;
    double victim_radius_;
    double dt_;
    double dubins_safety_;
    double sample_res_;
    double voronoi_res_;
    int    thin_iters_;
    int    dubins_discretizations_;
    std::string op_method_;

    ros::Publisher marker_pub_;
    ros::Publisher ref_pub_;
    ros::Publisher stats_pub_;

    const WorldModel* world_ = nullptr;
    std::atomic<bool> done_{false};

    comb::GeoMap             map_;
    comb::VoronoiRoadmap     vor_;
    std::vector<comb::Vec2>  waypoints_;
    std::vector<comb::RefSample> reference_;
    std::mutex               data_mtx_;
    double                   total_value_ = 0.0;
};
