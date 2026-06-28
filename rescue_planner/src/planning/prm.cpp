#include "planning/prm.hpp"

PRM::PRM(ros::NodeHandle& nh){
    world_ = nullptr;
}

void PRM::initialize(const WorldModel& world){
    world_ = &world;
}

void PRM::step(){
}

void PRM::visualize(){
}