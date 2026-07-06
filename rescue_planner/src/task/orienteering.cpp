#include "task/orienteering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace comb {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

OrienteeringResult solveExact(const std::vector<std::vector<double>>& dist,
                              const std::vector<double>& values, double Dmax) {
    const int n = static_cast<int>(values.size());
    OrienteeringResult res;
    const int START = 0;
    const int GATE = n + 1;

    if (dist[START][GATE] <= Dmax) {
        res.feasible = true;
        res.total_value = 0.0;
        res.total_length = dist[START][GATE];
    }
    if (n == 0) return res;

    const int full = 1 << n;
    std::vector<std::vector<double>> dp(full, std::vector<double>(n, kInf));

    for (int i = 0; i < n; ++i) {
        const double d = dist[START][i + 1];
        if (d < kInf) dp[1 << i][i] = d;
    }

    auto maskValue = [&](int mask) {
        double v = 0.0;
        for (int i = 0; i < n; ++i)
            if (mask & (1 << i)) v += values[i];
        return v;
    };

    for (int mask = 1; mask < full; ++mask) {
        for (int i = 0; i < n; ++i) {
            if (!(mask & (1 << i))) continue;
            const double base = dp[mask][i];
            if (base == kInf) continue;
            for (int j = 0; j < n; ++j) {
                if (mask & (1 << j)) continue;
                const double d = dist[i + 1][j + 1];
                if (d == kInf) continue;
                const int nmask = mask | (1 << j);
                const double nd = base + d;
                if (nd < dp[nmask][j]) dp[nmask][j] = nd;
            }
        }
    }

    for (int mask = 1; mask < full; ++mask) {
        const double val = maskValue(mask);
        for (int i = 0; i < n; ++i) {
            if (!(mask & (1 << i))) continue;
            if (dp[mask][i] == kInf) continue;
            const double dg = dist[i + 1][GATE];
            if (dg == kInf) continue;
            const double total = dp[mask][i] + dg;
            if (total > Dmax) continue;
            const bool better =
                (val > res.total_value) ||
                (val == res.total_value && total < res.total_length) ||
                !res.feasible;
            if (better) {
                res.feasible = true;
                res.total_value = val;
                res.total_length = total;
                std::vector<int> order;
                int cmask = mask, ci = i;
                while (cmask) {
                    order.push_back(ci);
                    const double cur = dp[cmask][ci];
                    const int pmask = cmask ^ (1 << ci);
                    if (pmask == 0) break;
                    int pj = -1;
                    for (int k = 0; k < n; ++k) {
                        if (!(pmask & (1 << k))) continue;
                        if (dp[pmask][k] == kInf) continue;
                        if (std::fabs(dp[pmask][k] + dist[k + 1][ci + 1] - cur) <
                            1e-9) {
                            pj = k;
                            break;
                        }
                    }
                    if (pj < 0) break;
                    cmask = pmask;
                    ci = pj;
                }
                std::reverse(order.begin(), order.end());
                res.victim_order = order;
            }
        }
    }
    return res;
}

double routeLength(const std::vector<std::vector<double>>& dist,
                   const std::vector<int>& order, int gate) {
    int prev = 0;
    double len = 0.0;
    for (int v : order) {
        len += dist[prev][v + 1];
        prev = v + 1;
    }
    len += dist[prev][gate];
    return len;
}

OrienteeringResult solveGreedy(const std::vector<std::vector<double>>& dist,
                               const std::vector<double>& values, double Dmax) {
    const int n = static_cast<int>(values.size());
    const int GATE = n + 1;
    OrienteeringResult res;
    res.feasible = (dist[0][GATE] <= Dmax);
    res.total_length = res.feasible ? dist[0][GATE] : kInf;

    std::vector<int> order;
    std::vector<bool> used(n, false);

    bool improved = true;
    while (improved) {
        improved = false;
        double bestRatio = -1.0;
        int bestV = -1, bestPos = -1;
        for (int v = 0; v < n; ++v) {
            if (used[v]) continue;
            for (int pos = 0; pos <= static_cast<int>(order.size()); ++pos) {
                std::vector<int> cand = order;
                cand.insert(cand.begin() + pos, v);
                const double len = routeLength(dist, cand, GATE);
                if (len > Dmax || std::isinf(len)) continue;
                const double extra = len - routeLength(dist, order, GATE);
                const double ratio =
                    values[v] / std::max(extra, 1e-6);
                if (ratio > bestRatio) {
                    bestRatio = ratio;
                    bestV = v;
                    bestPos = pos;
                }
            }
        }
        if (bestV >= 0) {
            order.insert(order.begin() + bestPos, bestV);
            used[bestV] = true;
            improved = true;
        }
    }

    // 2-opt can shorten the route without lowering value.
    bool opt = true;
    while (opt && order.size() >= 2) {
        opt = false;
        for (size_t a = 0; a + 1 < order.size(); ++a) {
            for (size_t b = a + 1; b < order.size(); ++b) {
                std::vector<int> cand = order;
                std::reverse(cand.begin() + a, cand.begin() + b + 1);
                if (routeLength(dist, cand, GATE) + 1e-9 <
                    routeLength(dist, order, GATE)) {
                    order = cand;
                    opt = true;
                }
            }
        }
    }

    if (!order.empty()) {
        const double len = routeLength(dist, order, GATE);
        if (len <= Dmax && !std::isinf(len)) {
            double val = 0.0;
            for (int v : order) val += values[v];
            res.feasible = true;
            res.victim_order = order;
            res.total_value = val;
            res.total_length = len;
        }
    }
    return res;
}
}  // namespace

OrienteeringResult solveOrienteering(
    const std::vector<std::vector<double>>& dist,
    const std::vector<double>& values, double Dmax, const std::string& method,
    int exact_limit) {
    const int n = static_cast<int>(values.size());
    if (method == "exact") return solveExact(dist, values, Dmax);
    if (method == "greedy") return solveGreedy(dist, values, Dmax);
    if (n <= exact_limit) return solveExact(dist, values, Dmax);
    return solveGreedy(dist, values, Dmax);
}

}  // namespace comb
