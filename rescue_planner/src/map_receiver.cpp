#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseArray.h>

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

    ROS_INFO_STREAM(
        "Stored "
        << world.obstacles.size()
        << " obstacles"
    );
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


int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");

    ros::NodeHandle nh;

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

    ROS_INFO("Map receiver started");

    ros::Rate rate(1);

    while(ros::ok()){
        ros::spinOnce();

        ROS_INFO_STREAM(
            "Obstacles: " << world.obstacles.size()
            << " | Victims: " << world.victims.size()
            << " | Gates: " << world.gates.size()
        );

        for(const auto& victim : world.victims){
            ROS_INFO_STREAM(
                "Victim at ("
                << victim.x
                << ", "
                << victim.y
                << ")"
            );
        }
        for(const auto& obs : world.obstacles)
        {
            ROS_INFO_STREAM(
                "Obstacle points: "
                << obs.polygon.points.size()
            );
        }


        rate.sleep();
    }
    return 0;
}