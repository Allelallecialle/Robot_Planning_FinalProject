#pragma once

#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "planner.hpp"
#include "world_model.hpp"
#include "task/roadmap_graph.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"

class RRT : public Planner{
public:

    explicit RRT(ros::NodeHandle& nh);
    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;

    RoadmapGraph buildRoadmapGraph() const;
    bool planning_done=false;
    void publishReference(const std::vector<comb::RefSample>& ref);

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
    std::vector<comb::RefSample> reference_;
    ros::Publisher ref_pub_;
    std::vector<int> selected_path_;
    RoadmapGraph roadmap_;

    int nearestNode(double x, double y);
    RRTNode steer(const RRTNode& nearest,double target_x,double target_y,double step_size);
};