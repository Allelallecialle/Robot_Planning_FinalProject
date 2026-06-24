#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseArray.h>

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

        //TODO: sostituire p (sample casuale) con rrt based choice del sample
        int nearest = nearestNode(tree, p.x, p.y);
        ROS_INFO_STREAM(
            "Nearest node: "
            << nearest
        );

        RRTNode node;
        node.x = p.x;
        node.y = p.y;
        node.parent = nearest;
        tree.push_back(node);
        ROS_INFO_STREAM(
        "Tree size: "
        << tree.size());

        rate.sleep();
    }
    return 0;
}