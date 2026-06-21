#include <ros/ros.h>
#include "world_model.hpp"

#include <obstacles_msgs/ObstacleArrayMsg.h>

void obstacleCallback(
    const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg)
{
    ROS_INFO_STREAM(
        "Received "
        << msg->obstacles.size()
        << " obstacles"
    );
}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");

    ros::NodeHandle nh;
    WorldModel world;

    ros::Subscriber obstacle_sub =
    nh.subscribe(
        "/obstacles",
        1,
        obstacleCallback
    );

    ROS_INFO("Map receiver started");
    ROS_INFO_STREAM(
        "Obstacles: "
        << world.obstacles_count
    );

    ros::Rate rate(1);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}