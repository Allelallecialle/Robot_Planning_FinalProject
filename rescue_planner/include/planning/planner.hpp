#pragma once
#include "world_model.hpp"
#include "utils/benchmark_metrics.hpp"

class Planner{
public:
    virtual void initialize(const WorldModel& world) = 0;
    virtual void step() = 0;
    virtual void visualize() = 0;
    virtual ~Planner() {}

    virtual bool isPlanningDone() const = 0;
    // Benchmark
    void setMetrics(RunMetrics* m)
    {
        metrics_ = m;
    }

protected:
    RunMetrics* metrics_ = nullptr;
};