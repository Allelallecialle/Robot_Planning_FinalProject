// Add an arbitrary (x,y) POI (start, victim, or gate) in an
// existing roadmap graph as a new node. Connects it to every already present
// node within radius that has a collision free straight line to it.
//
// This lets victim_mission.cpp treat all planners uniformly: whatever
// graph a planner produced (graph, tree, etc), the
// POIs are never part of that graph originally, they're mission-specific, not
// planning-specific. So Dijkstra can later treat them just like any other graph node.

#include "utils/roadmap_utils.hpp"

#include "collision_checker.hpp"

#include <cmath>

namespace roadmap
{

int addTemporaryNode(RoadmapGraph& graph, double x, double y, const WorldModel& world, double radius){
    GraphNode n;

    n.x=x;
    n.y=y;

    graph.nodes.push_back(n);
    graph.adjacency.emplace_back();

    int id=graph.nodes.size()-1;

    // Scan only nodes that existed before the new one (i < id)
    //The loop just to attaches the new node to the pre-existing roadmap
    for(int i=0;i<id;i++)
    {
        double dx=x-graph.nodes[i].x;
        double dy=y-graph.nodes[i].y;

        double d=std::sqrt(dx*dx+dy*dy);

        if(d>radius)     // out of radius so skip collision check
            continue;

        if(!isSegmentValid(
            x,
            y,
            graph.nodes[i].x,
            graph.nodes[i].y,
            world))
            continue;   // straight line would hit an obstacle so skip

        // Undirected edge: add both directions since the roadmap graph is meant to be traversed both ways by Dijkstra.
        graph.adjacency[id].push_back({i,d});
        graph.adjacency[i].push_back({id,d});
    }

    return id;
}

}