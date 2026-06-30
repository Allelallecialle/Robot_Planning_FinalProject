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
        double radius;        // geometric disk radius (fixed, ~0.5 m)
        double value = 0.0;   // scalar reward (weight), smuggled in the
                              // ObstacleMsg.radius field by send_victims.cpp
    };

    // Robot start pose (filled from /<robot>/odom). Needed by the
    // combinatorial planner; the sampling planners simply root at (0,0).
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
    int victims_timeout = 0;   // seconds, from /victims_timeout (0 => unlimited)

    // "first message arrived" flags so the combinatorial planner does not plan
    // on a half-initialised world.
    bool obstacles_ready = false;
    bool victims_ready = false;
    bool start_ready = false;
    bool timeout_ready = false;
};
