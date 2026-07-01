#include "trajectory/dubins.hpp"

#include <limits>

namespace comb {

static const double TWOPI = 2.0 * M_PI;

double mod2pi(double theta) {
    double out = theta;
    while (out < 0.0) out += TWOPI;
    while (out >= TWOPI) out -= TWOPI;
    return out;
}

// sinc(t) = sin(t)/t with Taylor fallback for small t (unnormalised sinc).
static double sinc(double t) {
    if (std::fabs(t) < 0.002) {
        return 1.0 - (t * t) / 6.0 * (1.0 - (t * t) / 20.0);
    }
    return std::sin(t) / t;
}

void circline(double s, double x0, double y0, double th0, double k,
              double& x, double& y, double& th) {
    x = x0 + s * sinc(k * s / 2.0) * std::cos(th0 + k * s / 2.0);
    y = y0 + s * sinc(k * s / 2.0) * std::sin(th0 + k * s / 2.0);
    th = mod2pi(th0 + k * s);
}

static DubinsArc makeArc(double x0, double y0, double th0, double k, double L) {
    DubinsArc arc;
    arc.x0 = x0; arc.y0 = y0; arc.th0 = th0; arc.k = k; arc.L = L;
    circline(L, x0, y0, th0, k, arc.xf, arc.yf, arc.thf);
    return arc;
}

// ---- scaling to/from the standard problem ----
struct Scaled { double th0, thf, Kmax, lambda; };

static Scaled scaleToStandard(double x0, double y0, double th0,
                              double xf, double yf, double thf, double Kmax) {
    const double dx = xf - x0;
    const double dy = yf - y0;
    const double phi = std::atan2(dy, dx);
    const double lambda = std::hypot(dx, dy) / 2.0;
    Scaled s;
    s.lambda = lambda;
    s.Kmax = Kmax * lambda;
    s.th0 = mod2pi(th0 - phi);
    s.thf = mod2pi(thf - phi);
    return s;
}

// ---- the six primitives. Each returns feasibility + three scaled lengths. ----
struct Prim { bool ok; double s1, s2, s3; };

static Prim LSL(double th0, double thf, double K) {
    const double invK = 1.0 / K;
    const double C = std::cos(thf) - std::cos(th0);
    const double S = 2 * K + std::sin(th0) - std::sin(thf);
    const double t1 = std::atan2(C, S);
    const double s1 = invK * mod2pi(t1 - th0);
    const double t2 = 2 + 4 * K * K - 2 * std::cos(th0 - thf) +
                      4 * K * (std::sin(th0) - std::sin(thf));
    if (t2 < 0) return {false, 0, 0, 0};
    const double s2 = invK * std::sqrt(t2);
    const double s3 = invK * mod2pi(thf - t1);
    return {true, s1, s2, s3};
}

static Prim RSR(double th0, double thf, double K) {
    const double invK = 1.0 / K;
    const double C = std::cos(th0) - std::cos(thf);
    const double S = 2 * K - std::sin(th0) + std::sin(thf);
    const double t1 = std::atan2(C, S);
    const double s1 = invK * mod2pi(th0 - t1);
    const double t2 = 2 + 4 * K * K - 2 * std::cos(th0 - thf) -
                      4 * K * (std::sin(th0) - std::sin(thf));
    if (t2 < 0) return {false, 0, 0, 0};
    const double s2 = invK * std::sqrt(t2);
    const double s3 = invK * mod2pi(t1 - thf);
    return {true, s1, s2, s3};
}

static Prim LSR(double th0, double thf, double K) {
    const double invK = 1.0 / K;
    const double C = std::cos(th0) + std::cos(thf);
    const double S = 2 * K + std::sin(th0) + std::sin(thf);
    const double t1 = std::atan2(-C, S);
    const double t3 = 4 * K * K - 2 + 2 * std::cos(th0 - thf) +
                      4 * K * (std::sin(th0) + std::sin(thf));
    if (t3 < 0) return {false, 0, 0, 0};
    const double s2 = invK * std::sqrt(t3);
    const double t2 = -std::atan2(-2.0, s2 * K);
    const double s1 = invK * mod2pi(t1 + t2 - th0);
    const double s3 = invK * mod2pi(t1 + t2 - thf);
    return {true, s1, s2, s3};
}

static Prim RSL(double th0, double thf, double K) {
    const double invK = 1.0 / K;
    const double C = std::cos(th0) + std::cos(thf);
    const double S = 2 * K - std::sin(th0) - std::sin(thf);
    const double t1 = std::atan2(C, S);
    const double t3 = 4 * K * K - 2 + 2 * std::cos(th0 - thf) -
                      4 * K * (std::sin(th0) + std::sin(thf));
    if (t3 < 0) return {false, 0, 0, 0};
    const double s2 = invK * std::sqrt(t3);
    const double t2 = std::atan2(2.0, s2 * K);
    const double s1 = invK * mod2pi(th0 - t1 + t2);
    const double s3 = invK * mod2pi(thf - t1 + t2);
    return {true, s1, s2, s3};
}

static Prim RLR(double th0, double thf, double K) {
    const double invK = 1.0 / K;
    const double C = std::cos(th0) - std::cos(thf);
    const double S = 2 * K - std::sin(th0) + std::sin(thf);
    const double t1 = std::atan2(C, S);
    const double t2 = 0.125 * (6 - 4 * K * K + 2 * std::cos(th0 - thf) +
                               4 * K * (std::sin(th0) - std::sin(thf)));
    if (std::fabs(t2) > 1) return {false, 0, 0, 0};
    const double s2 = invK * mod2pi(TWOPI - std::acos(t2));
    const double s1 = invK * mod2pi(th0 - t1 + 0.5 * s2 * K);
    const double s3 = invK * mod2pi(th0 - thf + K * (s2 - s1));
    return {true, s1, s2, s3};
}

static Prim LRL(double th0, double thf, double K) {
    const double invK = 1.0 / K;
    const double C = std::cos(thf) - std::cos(th0);
    const double S = 2 * K + std::sin(th0) - std::sin(thf);
    const double t1 = std::atan2(C, S);
    const double t2 = 0.125 * (6 - 4 * K * K + 2 * std::cos(th0 - thf) -
                               4 * K * (std::sin(th0) - std::sin(thf)));
    if (std::fabs(t2) > 1) return {false, 0, 0, 0};
    const double s2 = invK * mod2pi(TWOPI - std::acos(t2));
    const double s1 = invK * mod2pi(t1 - th0 + 0.5 * s2 * K);
    const double s3 = invK * mod2pi(thf - th0 + K * (s2 - s1));
    return {true, s1, s2, s3};
}

DubinsCurve dubinsShortestPath(double x0, double y0, double th0,
                               double xf, double yf, double thf, double Kmax) {
    DubinsCurve curve;
    // Coincident points have no defined Dubins manoeuvre.
    if (std::hypot(xf - x0, yf - y0) < 1e-9) return curve;

    const Scaled sc = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);

    using PrimFn = Prim (*)(double, double, double);
    const PrimFn prims[6] = {LSL, RSR, LSR, RSL, RLR, LRL};
    static const int ksigns[6][3] = {
        { 1,  0,  1},  // LSL
        {-1,  0, -1},  // RSR
        { 1,  0, -1},  // LSR
        {-1,  0,  1},  // RSL
        {-1,  1, -1},  // RLR
        { 1, -1,  1},  // LRL
    };

    int pidx = -1;
    double bestL = std::numeric_limits<double>::infinity();
    double bs1 = 0, bs2 = 0, bs3 = 0;
    for (int i = 0; i < 6; ++i) {
        const Prim p = prims[i](sc.th0, sc.thf, sc.Kmax);
        const double Lcur = p.s1 + p.s2 + p.s3;
        if (p.ok && Lcur < bestL) {
            bestL = Lcur;
            bs1 = p.s1; bs2 = p.s2; bs3 = p.s3;
            pidx = i;
        }
    }
    if (pidx < 0) return curve;

    // Scale lengths back and rebuild the three arcs at the real curvature.
    const double s1 = bs1 * sc.lambda;
    const double s2 = bs2 * sc.lambda;
    const double s3 = bs3 * sc.lambda;
    const double k1 = ksigns[pidx][0] * Kmax;
    const double k2 = ksigns[pidx][1] * Kmax;
    const double k3 = ksigns[pidx][2] * Kmax;

    curve.a1 = makeArc(x0, y0, th0, k1, s1);
    curve.a2 = makeArc(curve.a1.xf, curve.a1.yf, curve.a1.thf, k2, s2);
    curve.a3 = makeArc(curve.a2.xf, curve.a2.yf, curve.a2.thf, k3, s3);
    curve.L = curve.a1.L + curve.a2.L + curve.a3.L;
    curve.valid = true;
    return curve;
}

void appendDiscretizedDubins(const DubinsCurve& curve, double v_max, double dt,
                             double t_offset, std::vector<RefSample>& out) {
    if (!curve.valid) return;

    const std::array<double, 3> lengths{curve.a1.L, curve.a2.L, curve.a3.L};
    const std::array<double, 3> omegas{curve.a1.k * v_max,
                                       curve.a2.k * v_max,
                                       curve.a3.k * v_max};
    const double tf = (lengths[0] + lengths[1] + lengths[2]) / v_max;
    const double sw0 = lengths[0] / v_max;
    const double sw1 = (lengths[0] + lengths[1]) / v_max;

    // ZOH unicycle integration (matches unicycle.py): exact for piecewise
    // constant (v, omega) over each dt using sinc-based midpoint update.
    double x = curve.a1.x0, y = curve.a1.y0, th = curve.a1.th0;
    double t = 0.0;
    while (t <= tf + 1e-12) {
        double omega;
        if (t < sw0)      omega = omegas[0];
        else if (t < sw1) omega = omegas[1];
        else              omega = omegas[2];

        out.push_back({x, y, th, v_max, omega, t_offset + t});

        x += v_max * dt * sinc(omega * dt / 2.0) * std::cos(th + omega * dt / 2.0);
        y += v_max * dt * sinc(omega * dt / 2.0) * std::sin(th + omega * dt / 2.0);
        th += omega * dt;
        t += dt;
    }
}

}  // namespace comb
