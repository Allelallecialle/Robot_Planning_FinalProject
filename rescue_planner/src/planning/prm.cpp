#include "planning/prm.hpp"
#include "sampler.hpp"
#include "collision_checker.hpp"
#include "task/roadmap_algorithms.hpp"
#include "task/orienteering.hpp"
#include "utils/roadmap_utils.hpp"
#include "utils/geometry_utils.hpp"

#include <iostream>
#include <cmath>
#include <vector>
#include <loco_planning/Reference.h>
#include <thread>


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

    // wait enough generated nodes to run dijsktra
    if(graph.size() < 300)
        return;
    // Build a temporary roadmap
    RoadmapGraph roadmap = buildRoadmapGraph();
    int startNode = roadmap::addTemporaryNode(roadmap, world_->start.x, world_->start.y, *world_);
    std::vector<int> victimNodes;

    for(const auto& victim : world_->victims){
        victimNodes.push_back(roadmap::addTemporaryNode(roadmap, victim.x, victim.y, *world_));
    }
    int gateNode = roadmap::addTemporaryNode(roadmap, world_->gates[0].position.x, world_->gates[0].position.y, *world_);

    std::vector<int> poi;
    poi.push_back(startNode);
    for(int id : victimNodes){
        poi.push_back(id);
    }
    poi.push_back(gateNode);

    std::vector<std::vector<double>> D(
    poi.size(),
    std::vector<double>(poi.size()));
    std::vector<std::vector<int>> prevs(poi.size());

    for(size_t i=0;i<poi.size();i++)
    {
        std::vector<double> dist;

        roadmap::dijkstra(
            roadmap,
            poi[i],
            dist,
            prevs[i]);

        for(size_t j=0;j<poi.size();j++)
            D[i][j]=dist[poi[j]];
    }

    std::vector<double> values;
    for(const auto& v : world_->victims){
        values.push_back(v.value);
    }
    double budget = world_->victims_timeout * 0.3;        // v_max
    auto result = comb::solveOrienteering(D, values, budget, "auto");
    std::vector<int> poiOrder;

    poiOrder.push_back(0);

    for(int v : result.victim_order)
        poiOrder.push_back(v+1);

    poiOrder.push_back(poi.size()-1);

    ROS_INFO("Selected %lu victims",
         result.victim_order.size());
    ROS_INFO("Collected value %.2f",
             result.total_value);


    // ---------- reconstruct full graph path ----------
    std::vector<int> graphPath;

    for(size_t i = 0; i + 1 < poiOrder.size(); i++)
    {
        std::vector<int> part =
            roadmap::reconstructPath(
                poi[poiOrder[i]],
                poi[poiOrder[i+1]],
                prevs[poiOrder[i]]);

        if(part.empty())
            continue;

        if(graphPath.empty())
        {
            graphPath.insert(
                graphPath.end(),
                part.begin(),
                part.end());
        }
        else
        {
            graphPath.insert(
                graphPath.end(),
                part.begin()+1,
                part.end());
        }
    }

    // ---------- convert graph nodes to geometric waypoints ----------
    std::vector<comb::Vec2> waypoints;

    for(int nodeId : graphPath)
    {
        comb::Vec2 p;

        p.x = roadmap.nodes[nodeId].x;
        p.y = roadmap.nodes[nodeId].y;

        waypoints.push_back(p);
    }

    ROS_INFO("Waypoint count = %lu", waypoints.size());

    if(waypoints.size() < 2)
        return;

    std::vector<double> headings =
        comb::optimizeHeadings(
            waypoints,
            world_->start.yaw,
            0.0,
            1.0 / 0.35,
            72);

    reference_.clear();

    double tOffset = 0.0;

    for(size_t i=0;i+1<waypoints.size();i++)
    {
        comb::DubinsCurve curve =
            comb::dubinsShortestPath(
                waypoints[i].x,
                waypoints[i].y,
                headings[i],
                waypoints[i+1].x,
                waypoints[i+1].y,
                headings[i+1],
                1.0/0.35);

        if(!curve.valid)
            continue;

        std::vector<comb::RefSample> leg;

        comb::appendDiscretizedDubins(
            curve,
            0.3,
            0.01,
            tOffset,
            leg);

        if(reference_.empty())
        {
            reference_.insert(
                reference_.end(),
                leg.begin(),
                leg.end());
        }
        else
        {
            reference_.insert(
                reference_.end(),
                leg.begin()+1,
                leg.end());
        }

        if(!reference_.empty())
            tOffset = reference_.back().t + 0.01;
    }

    ROS_INFO("Reference samples = %lu", reference_.size());
    publishReference(reference_);
    planning_done = true;

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

void PRM::publishReference(const std::vector<comb::RefSample>& ref)
{
    if(ref.empty())
        return;

    ros::Publisher pub = ref_pub_;

    std::thread([pub, ref]()
    {
        ros::Rate rate(100);

        for(const auto& s : ref)
        {
            if(!ros::ok())
                return;

            loco_planning::Reference msg;

            msg.x_d = s.x;
            msg.y_d = s.y;
            msg.theta_d = s.theta;
            msg.v_d = s.v;
            msg.omega_d = s.omega;
            msg.plan_finished = false;

            pub.publish(msg);

            rate.sleep();
        }

        loco_planning::Reference last;

        last.x_d = ref.back().x;
        last.y_d = ref.back().y;
        last.theta_d = ref.back().theta;
        last.v_d = 0.0;
        last.omega_d = 0.0;
        last.plan_finished = true;

        pub.publish(last);

        ROS_INFO("Reference published.");
    }).detach();
}