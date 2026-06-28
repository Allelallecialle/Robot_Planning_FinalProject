#pragma once

#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "planner.hpp"
#include "world_model.hpp"

class PRM : public Planner{
public:

    explicit PRM(ros::NodeHandle& nh);
    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;

private:
    struct PRMNode{
        double x;
        double y;
        std::vector<int> neighbours;
    };

    std::vector<PRMNode> graph;
    ros::Publisher marker_pub_;
    const WorldModel* world_;

    std::vector<int> nearNodes(double x, double y, double radius);
    double distance(const PRMNode& a,const PRMNode& b);
};