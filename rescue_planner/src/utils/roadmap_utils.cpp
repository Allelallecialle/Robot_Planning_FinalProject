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

    for(int i=0;i<id;i++)
    {
        double dx=x-graph.nodes[i].x;
        double dy=y-graph.nodes[i].y;

        double d=std::sqrt(dx*dx+dy*dy);

        if(d>radius)
            continue;

        if(!isSegmentValid(
            x,
            y,
            graph.nodes[i].x,
            graph.nodes[i].y,
            world))
            continue;

        graph.adjacency[id].push_back({i,d});
        graph.adjacency[i].push_back({id,d});
    }

    return id;
}

}