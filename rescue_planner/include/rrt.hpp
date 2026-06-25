#pragma once
#include <vector>

struct RRTNode
{
    double x;
    double y;

    int parent;
};

int nearestNode(const std::vector<RRTNode>& tree, double x, double y);

RRTNode steer(
    const RRTNode& nearest,
    double target_x,
    double target_y,
    double step_size
);