#pragma once
#include <vector>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>

class WorldModel{
public:

    struct Obstacle{
        geometry_msgs::Polygon polygon;
        double radius;
    };

    struct Victim{
        double x;
        double y;
        double radius;
    };

    std::vector<Obstacle> obstacles;
    std::vector<Victim> victims;
    std::vector<geometry_msgs::Pose> gates;
    geometry_msgs::Polygon borders;
};