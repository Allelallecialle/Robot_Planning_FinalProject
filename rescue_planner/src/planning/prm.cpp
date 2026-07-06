#include "planning/prm.hpp"
#include "sampler.hpp"
#include "collision_checker.hpp"
#include "task/roadmap_algorithms.hpp"
#include "task/orienteering.hpp"
#include "utils/roadmap_utils.hpp"
#include "utils/geometry_utils.hpp"
#include "task/victim_mission.hpp"
#include "task/reference_generation.hpp"
#include "utils/reference_publisher.hpp"

#include <iostream>
#include <cmath>
#include <vector>
#include <loco_planning/Reference.h>
#include <thread>
#include <sstream>
#include <iomanip>


PRM::PRM(ros::NodeHandle& nh){
    world_ = nullptr;
    marker_pub_ =
        nh.advertise<visualization_msgs::Marker>(
            "/prm_graph",
            1);
    ref_pub_ =
        nh.advertise<loco_planning::Reference>(
            "/limo0/ref",
            10);
}

void PRM::initialize(const WorldModel& world){
    world_ = &world;
}

double PRM::distance(const PRMNode& a, const PRMNode& b){
    double dx = a.x - b.x;
    double dy = a.y - b.y;

    return std::sqrt(dx*dx + dy*dy);
}

std::vector<int> PRM::nearNodes(double x, double y, double radius){
    std::vector<int> result;

    for(size_t i=0;i<graph.size();i++){
        double dx = graph[i].x - x;
        double dy = graph[i].y - y;

        double d = std::sqrt(dx*dx + dy*dy);

        if(d<radius){
            result.push_back(i);
        }
    }

    return result;
}

void PRM::step(){
    if(planning_done){
        return;
    }

    // Do not sample/connect until the whole world has arrived. Otherwise nodes
    // and edges built in the window before /obstacles is received ignore the
    // obstacles and the roadmap ends up with edges that cut through them.
    if(!world_->obstacles_ready || !world_->victims_ready ||
       !world_->start_ready || !world_->timeout_ready ||
       world_->gates.empty() || world_->borders.points.size() < 3)
        return;

    // Build the full graph to the fixed node budget, then plan once.
    while(graph.size() < 500){
        SamplePoint p = sampleRandomPoint(*world_);
        if(!isPointValid(p.x, p.y, *world_))
            continue;

        PRMNode node;
        node.x = p.x;
        node.y = p.y;

        std::vector<int> near = nearNodes(node.x, node.y, 2.0);
        graph.push_back(node);
        int new_index = graph.size()-1;
        for(int idx : near){
            if(!isSegmentValid(graph[idx].x, graph[idx].y, node.x, node.y, *world_)){
                continue;
            }
            graph[idx].neighbours.push_back(new_index);
            graph[new_index].neighbours.push_back(idx);
        }
    }

    roadmap_ = buildRoadmapGraph();
    if(metrics_){
        metrics_->roadmap_nodes = roadmap_.nodes.size();

        int edges = 0;
        for(const auto& adj : roadmap_.adjacency)
            edges += adj.size();

        metrics_->roadmap_edges = edges;
    }

    auto plan = planSamplingMission(roadmap_, *world_, world_->start.yaw);
    const auto& mission = plan.mission;
    if(!mission.feasible){
        ROS_WARN("No feasible rescue mission found on the fixed node budget "
                 "(%lu nodes); not sampling further.", graph.size());
        planning_done = true;  // freeze at fixed budget
        return;
    }

    selected_path_ = mission.graph_path;

    ROS_INFO("Selected %lu victims", mission.selected_victims.size());
    ROS_INFO("Collected value %.2f", mission.collected_value);
    ROS_INFO("Waypoint count = %lu", mission.graph_path.size());

    if(metrics_){
        metrics_->victims = mission.selected_victims.size();
        metrics_->path_length = mission.total_length;
        metrics_->score = mission.collected_value;
        metrics_->success = mission.feasible;
    }

    reference_ = plan.reference;
    ROS_INFO("Reference samples = %lu", reference_.size());

    publishReference(reference_);
    planning_done = true;
}

void PRM::visualize(){
   visualization_msgs::Marker nodes;

    nodes.header.frame_id = "map";
    nodes.header.stamp = ros::Time::now();

    nodes.ns = "prm_nodes";
    nodes.id = 0;

    nodes.type = visualization_msgs::Marker::POINTS;
    nodes.action = visualization_msgs::Marker::ADD;

    nodes.scale.x = 0.15;
    nodes.scale.y = 0.15;

    nodes.color.a = 1.0;
    nodes.color.b = 1.0;

    for(const auto& n : graph){
        geometry_msgs::Point p;
        p.x = n.x;
        p.y = n.y;
        p.z = 0.0;

        nodes.points.push_back(p);
    }
    marker_pub_.publish(nodes);

    visualization_msgs::Marker edges;
    edges.header.frame_id = "map";
    edges.header.stamp = ros::Time::now();

    edges.ns = "prm_edges";
    edges.id = 1;

    edges.type = visualization_msgs::Marker::LINE_LIST;

    edges.action = visualization_msgs::Marker::ADD;

    edges.scale.x = 0.03;

    edges.color.a = 0.35; edges.color.r = 0.6; edges.color.g = 0.6; edges.color.b = 0.6;

    for(size_t i=0;i<graph.size();i++){
        for(int n : graph[i].neighbours)
        {
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;

            p1.x=graph[i].x;
            p1.y=graph[i].y;

            p2.x=graph[n].x;
            p2.y=graph[n].y;

            edges.points.push_back(p1);
            edges.points.push_back(p2);
        }
    }

    marker_pub_.publish(edges);
   visualization_msgs::Marker path;

    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    path.ns = "selected_path";
    path.id = 100;

    path.type = visualization_msgs::Marker::LINE_LIST;
    path.action = visualization_msgs::Marker::ADD;

    path.scale.x = 0.08;

    path.color.a = 1.0;
    path.color.r = 1.0;
    path.color.g = 0.0;
    path.color.b = 0.0;

   for(size_t i=0;i+1<selected_path_.size();i++)
    {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = roadmap_.nodes[selected_path_[i]].x;
        p1.y = roadmap_.nodes[selected_path_[i]].y;

        p2.x = roadmap_.nodes[selected_path_[i+1]].x;
        p2.y = roadmap_.nodes[selected_path_[i+1]].y;

        path.points.push_back(p1);
        path.points.push_back(p2);
    }

    marker_pub_.publish(path);
}

RoadmapGraph PRM::buildRoadmapGraph() const{
    RoadmapGraph g;

    g.nodes.reserve(graph.size());
    g.adjacency.resize(graph.size());

    for(const auto& node : graph)
    {
        GraphNode n;

        n.x = node.x;
        n.y = node.y;

        g.nodes.push_back(n);
    }

    for(size_t i = 0; i < graph.size(); i++)
    {
        for(int neigh : graph[i].neighbours)
        {
            GraphEdge e;

            e.to = neigh;

            double dx = graph[i].x - graph[neigh].x;
            double dy = graph[i].y - graph[neigh].y;

            e.cost = std::sqrt(dx*dx + dy*dy);

            g.adjacency[i].push_back(e);
        }
    }

    return g;
}

void PRM::publishReference(const std::vector<comb::RefSample>& ref){
    ros::Publisher pub = ref_pub_;
    publishRef(ref, pub);
}

bool PRM::isPlanningDone() const
{
    return planning_done;
}