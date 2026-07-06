#include "trajectory/dubins_dp.hpp"

#include <cmath>
#include <limits>

#include "trajectory/dubins.hpp"

namespace comb {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Cell {
    double th = 0.0;
    double len = kInf;
    int next = -1;
};
}  // namespace

std::vector<double> optimizeHeadings(const std::vector<Vec2>& pts,
                                     double start_yaw, double gate_yaw,
                                     double k_max, int discretizations) {
    const int P = static_cast<int>(pts.size());
    std::vector<double> result(P, 0.0);
    if (P == 0) return result;
    if (P == 1) { result[0] = start_yaw; return result; }

    std::vector<std::vector<Cell>> col(P);
    for (int i = 0; i < P; ++i) {
        if (i == 0) {
            col[i].push_back({start_yaw, kInf, -1});
        } else if (i == P - 1) {
            col[i].push_back({gate_yaw, kInf, -1});
        } else {
            for (int d = 0; d < discretizations; ++d) {
                const double th = 2.0 * M_PI * d / discretizations;
                col[i].push_back({th, kInf, -1});
            }
            // Seed with geometric tangents.
            const double th_in =
                std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
            const double th_out =
                std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
            col[i].push_back({th_in, kInf, -1});
            col[i].push_back({th_out, kInf, -1});
        }
    }

    for (auto& c : col[P - 1]) c.len = 0.0;

    for (int idx = P - 2; idx >= 0; --idx) {
        for (auto& ci : col[idx]) {
            ci.len = kInf;
            ci.next = -1;
            for (int j = 0; j < static_cast<int>(col[idx + 1].size()); ++j) {
                const Cell& cj = col[idx + 1][j];
                if (cj.len == kInf) continue;
                const DubinsCurve curve = dubinsShortestPath(
                    pts[idx].x, pts[idx].y, ci.th, pts[idx + 1].x,
                    pts[idx + 1].y, cj.th, k_max);
                if (!curve.valid) continue;
                const double total = curve.L + cj.len;
                if (total < ci.len) {
                    ci.len = total;
                    ci.next = j;
                }
            }
        }
    }

    int best = 0;
    double bestLen = kInf;
    for (int k = 0; k < static_cast<int>(col[0].size()); ++k) {
        if (col[0][k].len < bestLen) { bestLen = col[0][k].len; best = k; }
    }
    int cur = best;
    for (int idx = 0; idx < P; ++idx) {
        result[idx] = col[idx][cur].th;
        if (idx + 1 < P) cur = col[idx][cur].next;
        if (cur < 0) {
            for (int r = idx + 1; r < P; ++r)
                result[r] = (r == P - 1) ? gate_yaw : result[idx];
            break;
        }
    }
    return result;
}

}  // namespace comb
