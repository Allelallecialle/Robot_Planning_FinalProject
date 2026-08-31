#pragma once

#include <string>

struct RunMetrics
{
    std::string planner;
    // Free-form label for the current experiment group/config, e.g.
    // "budget_140s", "prm_nodes_1000", "voronoi_res_0.05" -- lets several
    // sweeps share one CSV (or several CSVs) and still be told apart when
    // aggregating, without having to relaunch with different filenames.
    std::string run_tag;

    double initialization_time = 0.0;
    double planning_time = 0.0;
    double time_budget = 0.0;

    int roadmap_nodes = 0;
    int roadmap_edges = 0;

    double path_length = 0.0;

    int victims = 0;
    double score = 0.0;

    bool success = false;
};