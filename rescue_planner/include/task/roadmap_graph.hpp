#pragma once
#include <vector>

struct GraphEdge
{
    int to;
    double cost;
};

struct GraphNode
{
    double x;
    double y;
};

struct RoadmapGraph
{
    std::vector<GraphNode> nodes;
    std::vector<std::vector<GraphEdge>> adjacency;
};