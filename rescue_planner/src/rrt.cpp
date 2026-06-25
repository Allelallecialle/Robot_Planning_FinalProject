#include "rrt.hpp"

#include <cmath>

int nearestNode(const std::vector<RRTNode>& tree, double x, double y){
    int best_index = 0;

    double best_distance = 1e9;

    for(size_t i = 0; i < tree.size(); i++)
    {
        double dx = tree[i].x - x;
        double dy = tree[i].y - y;

        double distance =
            dx * dx +
            dy * dy;

        if(distance < best_distance)
        {
            best_distance = distance;
            best_index = i;
        }
    }

    return best_index;
}

RRTNode steer(const RRTNode& nearest, double target_x, double target_y, double step_size){
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