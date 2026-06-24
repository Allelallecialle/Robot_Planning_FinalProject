#pragma once

#include "world_model.hpp"

struct SamplePoint
{
    double x;
    double y;
};

SamplePoint sampleRandomPoint(const WorldModel& world);