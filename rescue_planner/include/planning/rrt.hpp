#pragma once

#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "planner.hpp"
#include "world_model.hpp"

class RRT : public Planner
{
     explicit RRT(ros::NodeHandle& nh);

    void initialize(const WorldModel& world) override;

    void step() override;

    void visualize() override;

private:
    struct RRTNode
    {
        double x;
        double y;
        int parent;
    };

    std::vector<RRTNode> tree;
    const WorldModel* world_;
    ros::Publisher marker_pub_;

    int nearestNode(const std::vector<RRTNode>& tree, double x, double y);

    RRTNode steer(
        const RRTNode& nearest,
        double target_x,
        double target_y,
        double step_size
    );
};