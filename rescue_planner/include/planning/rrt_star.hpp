#pragma once
#include <ros/ros.h>
#include "planner.hpp"

class RRTStar : public Planner{
public:

    explicit RRTStar(ros::NodeHandle& nh);
    void initialize(const WorldModel& world) override;
    void step() override;
    void visualize() override;

private:

    ros::Publisher marker_pub_;
    const WorldModel* world_;
};
