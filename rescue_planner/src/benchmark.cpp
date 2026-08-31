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
#include "planning/rrg.hpp"
#include "planning/visibility_planner.hpp"
#include "planning/cell_decomp_planner.hpp"
#include "planning/voronoi_planner.hpp"
#include "utils/benchmark_metrics.hpp"

#include "world_model.hpp"
#include <fstream>

// `filename` now comes from the "output_csv" ROS param (see main()) instead
// of being hardcoded, so different experiment sweeps can be routed to
// different CSVs (e.g. budget_sweep.csv, prm_node_sweep.csv, ...) just by
// changing a launch argument, with no recompile. If the directory doesn't
// exist yet, this will fail to open the file -- create it once up front
// (`mkdir -p`) before launching a sweep into a new path.
void saveMetrics(const RunMetrics& m, const std::string& filename)
{
    bool header = false;

    std::ifstream in(filename);
    header = !in.good();
    in.close();

    std::ofstream file(filename,std::ios::app);

    if(!file.is_open()){
        ROS_ERROR_STREAM("Could not open benchmark CSV at [" << filename
                         << "] -- does the directory exist? Metrics NOT saved.");
        return;
    }

    if(header)
    {
        file
        << "planner,"
        << "run_tag,"
        << "init,"
        << "planning,"
        << "time_budget,"
        << "nodes,"
        << "edges,"
        << "path,"
        << "victims,"
        << "score,"
        << "success\n";
    }

    file
        << m.planner << ","
        << m.run_tag << ","
        << m.initialization_time << ","
        << m.planning_time << ","
        << m.time_budget << ","
        << m.roadmap_nodes << ","
        << m.roadmap_edges << ","
        << m.path_length << ","
        << m.victims << ","
        << m.score << ","
        << m.success << "\n";

    file.flush();
    file.close();

    ROS_INFO_STREAM("Metrics saved to " << filename);
}

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
    ros::init(argc, argv, "benchmark");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string planner_type;
    std::string robot_name;
    pnh.param<std::string>("robot_name", robot_name, std::string("limo0"));

    // Where to append this run's row, and how to label it. Both default to
    // the old hardcoded behaviour (one shared file, no tag) if unset, so
    // existing launch files keep working unchanged.
    std::string output_csv;
    std::string run_tag;
    pnh.param<std::string>(
        "output_csv", output_csv,
        std::string("/root/project_ros_ws/src/Robot_Planning_FinalProject/"
                    "rescue_planner/src/test_benchmark/benchmark.csv"));
    pnh.param<std::string>("run_tag", run_tag, std::string(""));

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

    RunMetrics metrics;
    metrics.planner = planner_type;
    metrics.run_tag = run_tag;
    bool initialized = false;
    ros::WallTime t0;

    if(planner_type=="prm"){
        planner = std::make_unique<PRM>(pnh);
        ROS_INFO("PRM SELECTED");
    }else if(planner_type=="rrt_star"){
        planner = std::make_unique<RRTStar>(pnh);
        ROS_INFO("RRT STAR SELECTED");
    }else if(planner_type=="rrg"){
        planner = std::make_unique<RRG>(pnh);
        ROS_INFO("RRG SELECTED");
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
    planner->setMetrics(&metrics);

    while(ros::ok()){
        ros::spinOnce();

        if(!initialized && world.obstacles_ready && world.victims_ready && world.start_ready && world.timeout_ready){
            t0 = ros::WallTime::now();
            planner->initialize(world);
            metrics.initialization_time = (ros::WallTime::now()-t0).toSec();
            metrics.time_budget = world.victims_timeout;
            initialized = true;
            ROS_INFO("Planner initialized.");
        }

        if(initialized){
            static bool planning_started = false;
            static bool planning_finished = false;

            if(!planning_started){
                planning_started = true;
                t0 = ros::WallTime::now();
            }

            planner->step();

            if (!planning_finished && planner->isPlanningDone())
            {
                planning_finished = true;

                metrics.planning_time =
                    (ros::WallTime::now() - t0).toSec();

                saveMetrics(metrics, output_csv);

                ROS_INFO("Benchmark completed.");
            }
        }

        planner->visualize();

        rate.sleep();
    }


    return 0;
}
