#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include "collision_checker.hpp"
#include "sampler.hpp"
#include "rrt.hpp"
#include "world_model.hpp"
WorldModel world;

void obstacleCallback(
    const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg)
{
    world.obstacles.clear();

    for(const auto& obs : msg->obstacles)
    {
        WorldModel::Obstacle obstacle;

        obstacle.polygon = obs.polygon;
        obstacle.radius = obs.radius;

        world.obstacles.push_back(obstacle);
    }
}

void victimCallback(
    const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg){
    world.victims.clear();

    for(const auto& victim_msg : msg->obstacles)
    {
        if(victim_msg.polygon.points.empty())
            continue;

        WorldModel::Victim victim;

        victim.x = victim_msg.polygon.points[0].x;
        victim.y = victim_msg.polygon.points[0].y;
        victim.radius = victim_msg.radius;

        world.victims.push_back(victim);
    }
}
void gatesCallback(
    const geometry_msgs::PoseArray::ConstPtr& msg){
    world.gates = msg->poses;
}
void bordersCallback(
    const geometry_msgs::Polygon::ConstPtr& msg){
    world.borders = *msg;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");

    ros::NodeHandle nh;
    std::vector<RRTNode> tree;

    ros::Publisher marker_pub =
    nh.advertise<visualization_msgs::Marker>(
        "/rrt_tree",
        1
    );

    ros::Subscriber obstacle_sub =
    nh.subscribe(
        "/obstacles",
        1,
        obstacleCallback
    );

    ros::Subscriber victim_sub =
    nh.subscribe(
        "/victims",
        1,
        victimCallback
    );

    ros::Subscriber gates_sub =
    nh.subscribe(
        "/gates",
        1,
        gatesCallback
    );

    ros::Subscriber borders_sub =
    nh.subscribe(
        "/map_borders",
        1,
        bordersCallback
    );

    ROS_INFO("Map receiver started");

    ros::Rate rate(1);

    while(ros::ok()){
        ros::spinOnce();

        SamplePoint p = sampleRandomPoint(world);
        ROS_INFO_STREAM(
            "Sample: "
            << p.x
            << ", "
            << p.y
        );

        if(tree.empty()){
            RRTNode root;
            root.x = 0.0;
            root.y = 0.0;
            root.parent = -1;
            tree.push_back(root);
            ROS_INFO_STREAM("Root node created");
        }

        int nearest = nearestNode(tree, p.x, p.y);
        ROS_INFO_STREAM(
            "Nearest node: "
            << nearest
        );

        RRTNode node =steer(tree[nearest], p.x, p.y, 0.5);
        node.parent = nearest;
        tree.push_back(node);
        ROS_INFO_STREAM("Nearest: ("<< tree[nearest].x<< ", "<< tree[nearest].y<< ")");
        ROS_INFO_STREAM("New node: ("<< node.x<< ", "<< node.y<< ")");

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

        rate.sleep();
    }
    return 0;
}