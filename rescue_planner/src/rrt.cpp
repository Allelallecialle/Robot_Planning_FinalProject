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