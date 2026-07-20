#include "planning/rrt.hpp"
#include "sampler.hpp"
#include "collision_checker.hpp"
#include "task/victim_mission.hpp"
#include "task/reference_generation.hpp"
#include "utils/reference_publisher.hpp"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <loco_planning/Reference.h>
#include <sstream>
#include <thread>

RRT::RRT(ros::NodeHandle& nh){
    world_ = nullptr;
    marker_pub_ =
        nh.advertise<visualization_msgs::Marker>(
            "/rrt_tree",
            1);
    ref_pub_ =
        nh.advertise<loco_planning::Reference>(
            "/limo0/ref",
            10);
}

void RRT::initialize(const WorldModel& world)
{
    world_ = &world;
    if(tree.empty()){
        RRTNode root;

        root.x = world.start.x;
        root.y = world.start.y;
        root.parent = -1;

        tree.push_back(root);

        ROS_INFO("Root created at (%.2f, %.2f)", root.x, root.y);
    }
}

int RRT::nearestNode(double x, double y){
    int best_index = 0;
    double best_distance = 1e9;

    for(size_t i = 0; i < tree.size(); i++){
        double dx = tree[i].x - x;
        double dy = tree[i].y - y;

        double distance = dx * dx + dy * dy;

        if(distance < best_distance){
            best_distance = distance;
            best_index = i;
        }
    }

    return best_index;
}

RRT::RRTNode RRT::steer(const RRTNode& nearest, double target_x, double target_y, double step_size){
    RRTNode new_node;
    double dx = target_x - nearest.x;
    double dy = target_y - nearest.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    if(distance < step_size){
        new_node.x = target_x;
        new_node.y = target_y;
    } else {
        new_node.x = nearest.x + step_size * dx / distance;
        new_node.y = nearest.y + step_size * dy / distance;
    }

    new_node.parent = -1;
    return new_node;
}

void RRT::step(){
    if(planning_done)
        return;

    // Do not sample/connect until the whole world has arrived. Otherwise nodes
    // and edges built in the window before /obstacles is received ignore the
    // obstacles and the roadmap ends up with edges that cut through them.
    if(!world_->obstacles_ready || !world_->victims_ready ||
       !world_->start_ready || !world_->timeout_ready ||
       world_->gates.empty() || world_->borders.points.size() < 3)
        return;

    // Build the full tree to the fixed node budget, then plan once.
//     while(tree.size() < 1500){
//         SamplePoint p = sampleRandomPoint(*world_);
//
//         int nearest = nearestNode(p.x,p.y);
//
//         RRTNode node = steer(tree[nearest], p.x, p.y, 1.0);
//
//         if(isSegmentValid(tree[nearest].x,tree[nearest].y,node.x,node.y,*world_)){
//             node.parent = nearest;
//             tree.push_back(node);
//         }
//     }
    if(tree.empty())  // root not planted yet (waiting on start_ready)
        return;

    static bool rng_seeded = false;
    if(!rng_seeded){ srand(time(nullptr)); rng_seeded = true; }

    std::vector<SamplePoint> goals;
    for(const auto& v : world_->victims) goals.push_back({v.x, v.y});
    goals.push_back({world_->gates[0].position.x, world_->gates[0].position.y});

    std::vector<bool> reached(goals.size(), false);
    const double goal_tolerance = 0.2;   // must be >= steer() step_size below
    const double goal_bias = 0.1;        // fraction of samples aimed at a target
    const std::size_t max_nodes = 1500;  // safety cap for unreachable targets

    auto allReached = [&](){
        for(bool r : reached) if(!r) return false;
        return true;
    };
    auto tryMarkReached = [&](int node_idx){
        for(std::size_t g = 0; g < goals.size(); ++g){
            if(reached[g]) continue;
            const double dx = tree[node_idx].x - goals[g].x;
            const double dy = tree[node_idx].y - goals[g].y;
            if(std::sqrt(dx*dx + dy*dy) < goal_tolerance &&
               isSegmentValid(tree[node_idx].x, tree[node_idx].y,
                              goals[g].x, goals[g].y, *world_)){
                reached[g] = true;
            }
        }
    };
    tryMarkReached(0);  // a target might already sit next to the start

    // to stop the tree growth when the goal is reached exchange the 2 while conditions:
    //while(tree.size() < max_nodes && !allReached()){

    // grow the tree until the set number of nodes is reached
    while(tree.size() < max_nodes){
        SamplePoint p;
        std::vector<std::size_t> pending;
        for(std::size_t g = 0; g < goals.size(); ++g)
            if(!reached[g]) pending.push_back(g);

        if(!pending.empty() && (double)rand() / RAND_MAX < goal_bias){
            p = goals[pending[rand() % pending.size()]];
        } else {
            p = sampleRandomPoint(*world_);
        }

        int nearest = nearestNode(p.x,p.y);

        RRTNode node = steer(tree[nearest], p.x, p.y, 1.0);

        if(isSegmentValid(tree[nearest].x,tree[nearest].y,node.x,node.y,*world_)){
            node.parent = nearest;
            tree.push_back(node);
            tryMarkReached(static_cast<int>(tree.size()) - 1);
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
    ROS_INFO("Mission feasible = %d", mission.feasible);
    ROS_INFO("Roadmap nodes = %lu", roadmap_.nodes.size());
    ROS_INFO("Graph path size = %lu", mission.graph_path.size());

    if(!mission.feasible){
        ROS_WARN("No feasible rescue mission found on the fixed node budget "
                 "(%lu nodes); not sampling further.", tree.size());
        planning_done = true;  // freeze at fixed budget
        return;
    }

    selected_path_ = mission.graph_path;

    ROS_INFO("Selected %lu victims",mission.selected_victims.size());
    ROS_INFO("Collected value %.2f", mission.collected_value);

    if(metrics_){
        metrics_->victims = mission.selected_victims.size();
        metrics_->score = mission.collected_value;
        metrics_->path_length = mission.total_length;
        metrics_->success = mission.feasible;
    }

    reference_ = plan.reference;

    ROS_INFO("Reference samples = %lu", reference_.size());

    publishReference(reference_);

    planning_done = true;
}

void RRT::visualize(){
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
    nodes.color.b = 1.0;

    for(const auto& n : tree){
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

    edges.ns = "rrt_edges";
    edges.id = 1;

    edges.type = visualization_msgs::Marker::LINE_LIST;

    edges.action = visualization_msgs::Marker::ADD;

    edges.scale.x = 0.03;

    edges.color.a = 0.35; edges.color.r = 0.6; edges.color.g = 0.6; edges.color.b = 0.6;

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

RoadmapGraph RRT::buildRoadmapGraph() const{
    RoadmapGraph g;

    g.nodes.reserve(tree.size());
    g.adjacency.resize(tree.size());

    for(const auto& node : tree)
    {
        GraphNode n;
        n.x = node.x;
        n.y = node.y;
        g.nodes.push_back(n);
    }

    for(size_t i = 1; i < tree.size(); i++)
    {
        GraphEdge e;

        e.to = tree[i].parent;

        double dx = tree[i].x - tree[tree[i].parent].x;
        double dy = tree[i].y - tree[tree[i].parent].y;

        e.cost = std::sqrt(dx*dx + dy*dy);

        g.adjacency[i].push_back(e);

        // Undirected for Dijkstra.
        GraphEdge back;
        back.to = i;
        back.cost = e.cost;

        g.adjacency[tree[i].parent].push_back(back);
    }

    return g;
}


void RRT::publishReference(const std::vector<comb::RefSample>& ref){
    ros::Publisher pub = ref_pub_;
    publishRef(ref, pub);
}

bool RRT::isPlanningDone() const
{
    return planning_done;
}
