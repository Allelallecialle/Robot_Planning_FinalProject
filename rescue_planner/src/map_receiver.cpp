#include <memory>

#include <ros/ros.h>

#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <cmath>

#include "planning/planner.hpp"
#include "planning/rrt.hpp"
#include "planning/prm.hpp"
#include "planning/rrt_star.hpp"
#include "planning/visibility_planner.hpp"
#include "planning/cell_decomp_planner.hpp"
#include "planning/voronoi_planner.hpp"

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
    world.obstacles_ready = true;
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
        // send_victims publishes reward in radius; physical radius is fixed.
        victim.value = victim_msg.radius;
        victim.radius = 0.5;

        world.victims.push_back(victim);
    }
    world.victims_ready = true;
}
void gatesCallback(
    const geometry_msgs::PoseArray::ConstPtr& msg){
    world.gates = msg->poses;
}
void bordersCallback(
    const geometry_msgs::Polygon::ConstPtr& msg){
    world.borders = *msg;
}
void odomCallback(
    const nav_msgs::Odometry::ConstPtr& msg){
    world.start.x = msg->pose.pose.position.x;
    world.start.y = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    const double t0 = 2.0 * (q.w * q.z + q.x * q.y);
    const double t1 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    world.start.yaw = std::atan2(t0, t1);
    world.start_ready = true;
}
void timeoutCallback(
    const std_msgs::Int32::ConstPtr& msg){
    world.victims_timeout = msg->data;
    world.timeout_ready = true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string planner_type;
    std::string robot_name;
    pnh.param<std::string>("robot_name", robot_name, std::string("limo0"));

    double robot_radius = 0.20;
    double safety_margin = 0.10;
    pnh.param("robot_radius", robot_radius, robot_radius);
    pnh.param("safety_margin", safety_margin, safety_margin);
    world.clearance = robot_radius + safety_margin;

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

    ros::Subscriber odom_sub =
    nh.subscribe(
        "/" + robot_name + "/odom",
        1,
        odomCallback
    );

    ros::Subscriber timeout_sub =
    nh.subscribe(
        "/victims_timeout",
        1,
        timeoutCallback
    );


    ROS_INFO("Map receiver started");
    ros::Rate rate(1500);

    pnh.param("planner_type", planner_type, std::string("rrt"));
    ROS_INFO_STREAM(
        "planner_type = ["
        << planner_type
        << "]"
    );

    std::unique_ptr<Planner> planner;

    if(planner_type=="prm"){
        planner = std::make_unique<PRM>(pnh);
        ROS_INFO("PRM SELECTED");
    }else if(planner_type=="rrt_star"){
        planner = std::make_unique<RRTStar>(pnh);
        ROS_INFO("RRT STAR SELECTED");
    }else if(planner_type=="visibility"){
        planner = std::make_unique<VisibilityPlanner>(pnh);
        ROS_INFO("VISIBILITY (COMBINATORIAL) SELECTED");
    }else if(planner_type=="cell_decomp"){
        planner = std::make_unique<CellDecompPlanner>(pnh);
        ROS_INFO("CELL DECOMPOSITION (COMBINATORIAL) SELECTED");
    }else if(planner_type=="cell_decomp_approx"){
        planner = std::make_unique<CellDecompApproxPlanner>(pnh);
        ROS_INFO("CELL DECOMPOSITION APPROX (COMBINATORIAL) SELECTED");
    }else if(planner_type=="voronoi"){
        planner = std::make_unique<VoronoiPlanner>(pnh);
        ROS_INFO("VORONOI (COMBINATORIAL) SELECTED");
    }else{
        planner = std::make_unique<RRT>(pnh);
        ROS_INFO("RRT SELECTED");
    }

    while(ros::ok()){
        ros::spinOnce();

        planner->initialize(world);
        planner->step();
        planner->visualize();

        rate.sleep();
    }
    return 0;
}
