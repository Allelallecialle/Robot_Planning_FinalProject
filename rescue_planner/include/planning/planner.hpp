#pragma once
#include "world_model.hpp"

class Planner{
public:
    virtual void initialize(const WorldModel& world) = 0;
    virtual void step() = 0;
    virtual void visualize() = 0;
    virtual ~Planner() {}

    virtual bool isPlanningDone() const = 0;
};