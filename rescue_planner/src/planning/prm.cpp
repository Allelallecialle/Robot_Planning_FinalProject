#include "planning/prm.hpp"
#include "sampler.hpp"
#include "collision_checker.hpp"
#include <cmath>
#include <vector>


PRM::PRM(ros::NodeHandle& nh){
    world_ = nullptr;
    marker_pub_ =
        nh.advertise<visualization_msgs::Marker>(
            "/prm_graph",
            1);
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
    SamplePoint p = sampleRandomPoint(*world_);
    if(!isPointValid(p.x, p.y, *world_))
        return;

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

void PRM::visualize(){

    // to draw tree nodes on map
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
    nodes.color.g = 1.0;

    for(const auto& n : graph){
        geometry_msgs::Point p;
        p.x = n.x;
        p.y = n.y;
        p.z = 0.0;

        nodes.points.push_back(p);
    }
    marker_pub_.publish(nodes);

    //here edges
    visualization_msgs::Marker edges;
    edges.header.frame_id = "map";
    edges.header.stamp = ros::Time::now();

    edges.ns = "prm_edges";
    edges.id = 1;

    edges.type = visualization_msgs::Marker::LINE_LIST;

    edges.action = visualization_msgs::Marker::ADD;

    edges.scale.x = 0.03;

    edges.color.a = 1.0;
    edges.color.r = 1.0;

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
   //-----
}