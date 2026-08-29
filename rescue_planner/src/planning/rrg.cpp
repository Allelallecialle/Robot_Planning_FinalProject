#include "planning/rrg.hpp"
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

RRG::RRG(ros::NodeHandle& nh){
    world_ = nullptr;
    marker_pub_ =
        nh.advertise<visualization_msgs::Marker>(
            "/rrg_tree",
            1);
    ref_pub_ =
        nh.advertise<loco_planning::Reference>(
            "/limo0/ref",
            10);
}

void RRG::initialize(const WorldModel& world)
{
    world_ = &world;
    // Wait for odometry: rooting before start_ready would plant the graph at
    // the Pose{} default (0,0), not the robot's real starting position.
    if(tree.empty() && world.start_ready){
        RRGNode root;

        root.x = world.start.x;
        root.y = world.start.y;
        root.parent = -1;

        tree.push_back(root);
        adjacency_.emplace_back();  // adjacency row for the root node

        ROS_INFO("RRG root created at (%.2f, %.2f)", root.x, root.y);
    }
}

int RRG::nearestNode(double x, double y){
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

RRG::RRGNode RRG::steer(const RRGNode& nearest, double target_x, double target_y, double step_size){
    RRGNode new_node;
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

void RRG::step(){
    if(planning_done)
        return;

    // Wait until the whole world has arrived
    if(!world_->obstacles_ready || !world_->victims_ready ||
       !world_->start_ready || !world_->timeout_ready ||
       world_->gates.empty() || world_->borders.points.size() < 3)
        return;

    if(tree.empty())  // root not planted yet (waiting on start_ready)
        return;

    // Grow toward the mission targets (victims + gate), same
    // goal biased sampling idea as RRT/RRT* but generalised to multiple targets
    // since this is a multigoal orienteering mission and not a single goal query
    static bool rng_seeded = false;
    if(!rng_seeded){
        srand(time(nullptr));
        rng_seeded = true;
    }

    // Intialize the vector of goals (gates + victims). Used later to check if they're all connected to the tree
    std::vector<SamplePoint> goals;
    for(const auto& v : world_->victims) goals.push_back({v.x, v.y});
    goals.push_back({world_->gates[0].position.x, world_->gates[0].position.y});

    std::vector<bool> reached(goals.size(), false); // Set all goals to reached = false. When true it means: a tree node is close enough to the considered goal and the final segment to it is collision free
    const double goal_tolerance = 1.0;      // The radius distance for reaching the goal mentioned above
    const double goal_bias = 0.1;       // Percentage of samples aimed at a target. So the tree is encouraged to grow toward mission relevant locations
    const std::size_t target_nodes = 1500;  // Set number of nodes to grow. Safety cap for unreachable targets

    auto allReached = [&](){
        for(bool r : reached) if(!r) return false;
        return true;
    };
    // Check unreached targets against the newly added node
    auto tryMarkReached = [&](int node_idx){
        for(std::size_t g = 0; g < goals.size(); ++g){
            if(reached[g]) continue;
            const double dx = tree[node_idx].x - goals[g].x;
            const double dy = tree[node_idx].y - goals[g].y;
            // Check radius distance from closest node + collision free of the segment
            if(std::sqrt(dx*dx + dy*dy) < goal_tolerance &&
               isSegmentValid(tree[node_idx].x, tree[node_idx].y,
                              goals[g].x, goals[g].y, *world_)){
                reached[g] = true;
            }
        }
    };
    tryMarkReached(0);  // a target might already sit next to the start

    // RRG connection radius:
    //     r_n = min( gamma * (log(n)/n)^(1/d), eta )
    // where n = current node count, d = 2 (planar), eta = the fixed maximum
    // steer step size (the radius never exceeds the longest edge the graph
    // can create anyway), and gamma must exceed the theoretical minimum
    //     gamma* = 2*(1+1/d)^(1/d) * (mu_free/zeta_d)^(1/d)
    // for the random geometric graph to be connected almost surely as
    // n -> infinity (mu_free = Lebesgue measure of free space, zeta_d =
    // volume of the unit ball in R^d, i.e. pi for d=2). mu_free is
    // approximated by the map border polygon's area (ignoring obstacles):
    // this OVER-estimates the true free-space measure, which only makes
    // gamma/r_n larger than strictly required -- safe, never unsafe.
    const double eta = 1.0;  // must match steer()'s step_size below
    std::vector<comb::Vec2> borderPts;
    for(const auto& p : world_->borders.points) borderPts.push_back({p.x, p.y});
    const double mu_free = std::fabs(comb::polygonSignedArea(borderPts));
    constexpr int d = 2;
    const double zeta_d = M_PI;  // volume of the unit ball in R^2
    const double gamma_star =
        2.0 * std::pow(1.0 + 1.0 / d, 1.0 / d) * std::pow(mu_free / zeta_d, 1.0 / d);
    const double gamma_rrg = 1.5 * gamma_star;  // safety margin above the theoretical minimum

    // Note: with a small eta (short steer steps) and a map this size, r_n
    // below will be pinned at the eta cap for essentially the whole growth
    // (the shrinking (log n/n)^(1/d) term only drops below eta once n is far
    // larger than our node budgets) -- expected and still correct, it just
    // means in our regime this behaves like a fixed-radius RRG with radius eta.
    while(tree.size() < target_nodes){
        SamplePoint p;
        std::vector<std::size_t> pending;
        for(std::size_t g = 0; g < goals.size(); ++g)
            if(!reached[g]) pending.push_back(g);

        if(!pending.empty() && (double)rand() / RAND_MAX < goal_bias){
            p = goals[pending[rand() % pending.size()]];
        } else {
            p = sampleRandomPoint(*world_);
        }

        const int nearest = nearestNode(p.x,p.y);

        RRGNode node = steer(tree[nearest], p.x, p.y, eta);

        if(!isSegmentValid(tree[nearest].x,tree[nearest].y,node.x,node.y,*world_))
            continue;

        node.parent = nearest;  // kept only for visualize(), adjacency_ is the actual graph Dijkstra will use
        tree.push_back(node);
        adjacency_.emplace_back();

        const int newIdx = static_cast<int>(tree.size()) - 1;
        const double dNearest =
            std::hypot(node.x - tree[nearest].x, node.y - tree[nearest].y);
        adjacency_[newIdx].push_back({nearest, dNearest});
        adjacency_[nearest].push_back({newIdx, dNearest});

        // The actual RRG step: connect the new node to every other existing
        // node within the shrinking radius r_n and has collision free segment
        //Makes it a graph instead of a tree
        const double n = std::max<double>(tree.size(), 2.0);
        const double r_n = std::min(gamma_rrg * std::pow(std::log(n) / n, 1.0 / d), eta);

        for(int i = 0; i < newIdx; ++i){
            if(i == nearest) continue;  // already connected above

            const double dx = tree[i].x - node.x;
            const double dy = tree[i].y - node.y;
            const double dd = std::sqrt(dx*dx + dy*dy);
            if(dd > r_n) continue;

            if(!isSegmentValid(tree[i].x, tree[i].y, node.x, node.y, *world_))
                continue;

            adjacency_[newIdx].push_back({i, dd});
            adjacency_[i].push_back({newIdx, dd});
        }

        tryMarkReached(newIdx);
    }

    ROS_INFO("RRG grown to %lu nodes, all %lu targets reached = %d",
             tree.size(), goals.size(), allReached());

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

void RRG::visualize(){
   visualization_msgs::Marker nodes;

    nodes.header.frame_id = "map";
    nodes.header.stamp = ros::Time::now();

    nodes.ns = "rrg_nodes";
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

    // Render the FULL graph (adjacency_), not just parent-pointer edges --
    // unlike RRT/RRT*'s visualize(), this actually shows every RRG
    // connection, not a spanning-tree subset of it.
    visualization_msgs::Marker edges;
    edges.header.frame_id = "map";
    edges.header.stamp = ros::Time::now();

    edges.ns = "rrg_edges";
    edges.id = 1;

    edges.type = visualization_msgs::Marker::LINE_LIST;

    edges.action = visualization_msgs::Marker::ADD;

    edges.scale.x = 0.03;

    edges.color.a = 0.35; edges.color.r = 0.6; edges.color.g = 0.6; edges.color.b = 0.6;

    for(size_t i = 0; i < adjacency_.size(); i++){
        for(const auto& e : adjacency_[i]){
            if(static_cast<size_t>(e.to) < i) continue;  // undirected: draw each edge once

            geometry_msgs::Point p1;
            geometry_msgs::Point p2;

            p1.x = tree[i].x;
            p1.y = tree[i].y;

            p2.x = tree[e.to].x;
            p2.y = tree[e.to].y;

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

RoadmapGraph RRG::buildRoadmapGraph() const{
    RoadmapGraph g;

    g.nodes.reserve(tree.size());

    for(const auto& node : tree)
    {
        GraphNode n;
        n.x = node.x;
        n.y = node.y;
        g.nodes.push_back(n);
    }

    // The edges were already built incrementally during step(): every
    // insertion connects the new node to its nearest node AND to every other
    // node within the shrinking connection radius r_n that has a
    // collision-free line of sight. Nothing left to compute here -- just
    // hand over the adjacency exactly as grown. No post-hoc densification
    // pass needed (unlike RRT/RRT*), since this planner builds a properly
    // connected graph natively.
    g.adjacency = adjacency_;

    return g;
}

void RRG::publishReference(const std::vector<comb::RefSample>& ref){
    ros::Publisher pub = ref_pub_;
    publishRef(ref, pub);
}

bool RRG::isPlanningDone() const
{
    return planning_done;
}
