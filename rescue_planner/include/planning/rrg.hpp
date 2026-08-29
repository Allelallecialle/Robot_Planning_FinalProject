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
// Algorithms for Optimal Motion Planning" (2011). Structurally identical
// growth procedure to RRT (same nearestNode/steer/goal-biased sampling), but
// every accepted node connects to ALL nearby collision-free nodes within a
// shrinking radius r_n, not just its nearest neighbor -- producing a properly
// connected roadmap graph natively, instead of a bare tree that needs a
// separate densification pass to be usable for multi-POI Dijkstra queries
// (see RRT/RRT* for that patched-on approach; this planner is the "do it
// right from the start" alternative).
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
        int parent;  // kept ONLY for visualize()'s tree rendering; the real
                     // graph structure lives in adjacency_, not in parent
                     // pointers (unlike RRT/RRT*, this is a graph, not a tree)
    };

    std::vector<RRGNode> tree;
    const WorldModel* world_;
    ros::Publisher marker_pub_;
    std::vector<comb::RefSample> reference_;
    ros::Publisher ref_pub_;
    std::vector<int> selected_path_;
    RoadmapGraph roadmap_;
    // RRG adjacency, built INCREMENTALLY during step(): each new node
    // connects to its nearest node AND to every other node within the
    // shrinking connection radius r_n that has a collision-free line of
    // sight. This IS the graph handed to Dijkstra
    std::vector<std::vector<GraphEdge>> adjacency_;

    int nearestNode(double x, double y);
    RRGNode steer(const RRGNode& nearest,double target_x,double target_y,double step_size);
};
