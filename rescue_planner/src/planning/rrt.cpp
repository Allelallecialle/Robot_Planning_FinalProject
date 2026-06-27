#include "planning/rrt.hpp"
#include "sampler.hpp"
#include "collision_checker.hpp"
#include <cmath>

RRT::RRT(ros::NodeHandle& nh){
    world_ = nullptr;
    marker_pub_ =
        nh.advertise<visualization_msgs::Marker>(
            "/rrt_tree",
            1
        );
}

void RRT::initialize(const WorldModel& world)
{
    world_ = &world;
    if(tree.empty()){
        RRTNode root;

        root.x = 0;
        root.y = 0;
        root.parent = -1;

        tree.push_back(root);

        ROS_INFO("Root created");
    }
}

int RRT::nearestNode(double x, double y){
    int best_index = 0;

    double best_distance = 1e9;

    for(size_t i = 0; i < tree.size(); i++)
    {
        double dx = tree[i].x - x;
        double dy = tree[i].y - y;

        double distance =
            dx * dx +
            dy * dy;

        if(distance < best_distance)
        {
            best_distance = distance;
            best_index = i;
        }
    }

    return best_index;
}

int RRT::steer(const RRTNode& nearest, double target_x, double target_y, double step_size){
    RRTNode new_node;
    double dx = target_x - nearest.x;
    double dy = target_y - nearest.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    if(distance < step_size){
        new_node.x = target_x;
        new_node.y = target_y;
    } else {
        new_node.x = nearest.x + step_size * dx / distance;
        new_node.y = nearest.y + step_size * dy / distance;
    }

    new_node.parent = -1;
    return new_node;
}

void RRT::step(){
    SamplePoint p =
        sampleRandomPoint(*world_);

    int nearest =
        nearestNode(p.x,p.y);

    RRTNode node =
        steer(tree[nearest],
              p.x,
              p.y,
              1.0);

    if(isSegmentValid(
        tree[nearest].x,
        tree[nearest].y,
        node.x,
        node.y,
        *world_))
    {
        node.parent = nearest;
        tree.push_back(node);
    }
}

void RRT::visualize(){
   ros::Publisher marker_pub_;
   RRT(ros::NodeHandle& nh);
   marker_pub_ =
        nh.advertise<visualization_msgs::Marker>(
        "/rrt_tree",
        1);

   // to draw tree nodes on map
   visualization_msgs::Marker nodes;

    nodes.header.frame_id = "map";
    nodes.header.stamp = ros::Time::now();

    nodes.ns = "rrt_nodes";
    nodes.id = 0;

    nodes.type = visualization_msgs::Marker::POINTS;
    nodes.action = visualization_msgs::Marker::ADD;

    nodes.scale.x = 0.15;
    nodes.scale.y = 0.15;

    nodes.color.a = 1.0;
    nodes.color.g = 1.0;

    for(const auto& n : tree){
        geometry_msgs::Point p;
        p.x = n.x;
        p.y = n.y;
        p.z = 0.0;

        nodes.points.push_back(p);
    }
    marker_pub.publish(nodes);

    //here edges
    visualization_msgs::Marker edges;
    edges.header.frame_id = "map";
    edges.header.stamp = ros::Time::now();

    edges.ns = "rrt_edges";
    edges.id = 1;

    edges.type = visualization_msgs::Marker::LINE_LIST;

    edges.action = visualization_msgs::Marker::ADD;

    edges.scale.x = 0.03;

    edges.color.a = 1.0;
    edges.color.r = 1.0;

    for(size_t i = 1; i < tree.size(); i++)
    {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = tree[i].x;
        p1.y = tree[i].y;

        int parent = tree[i].parent;

        p2.x = tree[parent].x;
        p2.y = tree[parent].y;

        edges.points.push_back(p1);
        edges.points.push_back(p2);
    }

    marker_pub.publish(edges);
   //-----
}
