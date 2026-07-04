#pragma once
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "planner.hpp"
#include "world_model.hpp"
#include "task/roadmap_graph.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"

class RRTStar : public Planner{
public:

    explicit RRTStar(ros::NodeHandle& nh);
    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;

    std::vector<comb::RefSample> reference_;
    bool planning_done=false;
    RoadmapGraph buildRoadmapGraph() const;
    void publishReference(const std::vector<comb::RefSample>& ref);

private:
    struct RRTNode
    {
        double x;
        double y;
        int parent;
        double cost;
    };

    std::vector<RRTNode> tree;
    const WorldModel* world_;
    ros::Publisher marker_pub_;
    ros::Publisher ref_pub_;

    int nearestNode(double x,double y);
    RRTNode steer(const RRTNode& nearest, double target_x, double target_y, double step_size);

    std::vector<int> nearNodes(double x, double y, double radius);
    double distance(const RRTNode& a, const RRTNode& b);
    void rewire(int new_node, const std::vector<int>& near);

};
