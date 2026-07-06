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
        double value = 0.0;  // stored in ObstacleMsg.radius by send_victims.cpp
    };

    struct Pose{
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    std::vector<Obstacle> obstacles;
    std::vector<Victim> victims;
    std::vector<geometry_msgs::Pose> gates;
    geometry_msgs::Polygon borders;

    Pose start;
    int victims_timeout = 0;

    double clearance = 0.0;

    bool obstacles_ready = false;
    bool victims_ready = false;
    bool start_ready = false;
    bool timeout_ready = false;
};
