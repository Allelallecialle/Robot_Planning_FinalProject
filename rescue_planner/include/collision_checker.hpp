#pragma once

#include "world_model.hpp"

bool isInsideMap(double x,double y,const geometry_msgs::Polygon& polygon);

bool isPointValid(double x, double y, const WorldModel& world);

bool isSegmentValid(double x1, double y1, double x2, double y2, const WorldModel& world);