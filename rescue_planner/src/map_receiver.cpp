#include <memory>

#include <ros/ros.h>

#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseArray.h>

#include "planning/planner.hpp"
#include "planning/rrt.hpp"
#include "planning/prm.hpp"
#include "planning/rrt_star.hpp"

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
    std::string planner_type;

    /*
    ros::Publisher marker_pub =
    nh.advertise<visualization_msgs::Marker>(
        "/rrt_tree",
        1
    );*/

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
    ros::Rate rate(100);

    nh.param("planner_type",
             planner_type,
             std::string("rrt"));

    std::unique_ptr<Planner> planner;

    if(planner_type=="prm")
        planner = std::make_unique<PRM>(nh);

    else if(planner_type=="rrt_star")
        planner = std::make_unique<RRTStar>(nh);

    else
        planner = std::make_unique<RRT>(nh);

    while(ros::ok())
    {
        ros::spinOnce();
        planner->initialize(world);
        planner->step();
        planner->visualize();

        rate.sleep();
    }
    }
    return 0;
}