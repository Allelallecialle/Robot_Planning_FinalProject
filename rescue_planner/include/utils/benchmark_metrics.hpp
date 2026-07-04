#pragma once

#include <string>

struct RunMetrics
{
    std::string planner;

    double initialization_time = 0.0;
    double planning_time = 0.0;
    double time_budget = 0.0;

    int roadmap_nodes = 0;
    int roadmap_edges = 0;

    double path_length = 0.0;

    int victims = 0;

    bool success = false;
};