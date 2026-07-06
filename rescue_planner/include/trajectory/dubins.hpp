#pragma once

#include <vector>
#include <array>
#include <cmath>

namespace comb {

struct DubinsArc {
    double x0, y0, th0;
    double k;
    double L;
    double xf, yf, thf;
};

struct DubinsCurve {
    bool   valid = false;
    DubinsArc a1, a2, a3;
    double L = 0.0;
};

struct RefSample {
    double x, y, theta, v, omega, t;
};

DubinsCurve dubinsShortestPath(double x0, double y0, double th0,
                               double xf, double yf, double thf, double Kmax);
void appendDiscretizedDubins(const DubinsCurve& curve, double v_max, double dt,
                             double t_offset, std::vector<RefSample>& out);
void circline(double s, double x0, double y0, double th0, double k,
              double& x, double& y, double& th);
double mod2pi(double theta);

}  // namespace comb
