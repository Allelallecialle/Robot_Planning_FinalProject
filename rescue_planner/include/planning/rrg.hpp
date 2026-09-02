#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "planner.hpp"
#include "world_model.hpp"
#include "task/roadmap_graph.hpp"
#include "trajectory/dubins.hpp"
#include "trajectory/dubins_dp.hpp"

// RRG (Rapidly-exploring Random Graph), Karaman & Frazzoli, "Sampling-based
// Algorithms for Optimal Motion Planning". Structurally identical
// growth procedure to RRT (same nearestNode/steer/goal-biased sampling), but
// every accepted node connects to all nearby collision free nodes within a
// shrinking radius r_n
class RRG : public Planner{
public:

    explicit RRG(ros::NodeHandle& nh);
    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;
    bool isPlanningDone() const override;

    RoadmapGraph buildRoadmapGraph() const;
    bool planning_done=false;
    void publishReference(const std::vector<comb::RefSample>& ref);

private:
    struct RRGNode
    {
        double x;
        double y;
        int parent;  // kept for visualize()'s tree rendering. The real
                     // graph structure lives in adjacency_, this is a graph, not a tree
    };

    std::vector<RRGNode> tree;
    const WorldModel* world_;
    ros::Publisher marker_pub_;
    std::vector<comb::RefSample> reference_;
    ros::Publisher ref_pub_;
    std::vector<int> selected_path_;
    RoadmapGraph roadmap_;
    std::vector<std::vector<GraphEdge>> adjacency_;

    int nearestNode(double x, double y);
    RRGNode steer(const RRGNode& nearest,double target_x,double target_y,double step_size);
};
