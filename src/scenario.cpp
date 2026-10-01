#include "aim/planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace aim {
double Vehicle::radius() const {
  // Circumscribed body disc. Eq.(10b)'s printed subscripts do not cover the
  // rectangle in Fig.2; use its geometric half diagonal with the Eq.(10a)
  // center.
  return .5 * std::hypot(LW + LF + LR, LB);
}
double wrap(double t) { return std::atan2(std::sin(t), std::cos(t)); }
Point center(const Vehicle &v, const State &q) {
  double d = v.center_offset();
  return {q[X] + d * cos(q[Theta]), q[Y] + d * sin(q[Theta])};
}
Scenario crossing_example() {
  Scenario s;
  s.workspace = {-310, 310, -310, 310};
  // Four unequal corner berms form an open, lane-free intersection and legs.
  s.obstacles = {{-310, -24, -310, -22},
                 {24, 310, -310, -25},
                 {-310, -27, 24, 310},
                 {26, 310, 27, 310}};
  // Ten heterogeneous CAVs enter from all four legs. This is one joint NLP,
  // with no imposed crossing order or precomputed final trajectories.
  const std::array<std::array<double, 4>, 3> geometry{{
      {5.73, 1.71, 1.90, 3.50}, {5.30, 1.50, 1.60, 3.20},
      {4.90, 1.30, 1.50, 3.00}}};
  for (int i = 0; i < 10; ++i) {
    Vehicle v;
    auto g = geometry[i % 3];
    v.LW = g[0]; v.LF = g[1]; v.LR = g[2]; v.LB = g[3];
    v.heading = (i % 4 == 3 ? -1 : i % 4) * pi / 2;
    double ux = cos(v.heading), uy = sin(v.heading);
    double distance = 30 + 11.5 * i;
    v.initial[X] = -distance * ux + 12 * uy;
    v.initial[Y] = -distance * uy - 12 * ux;
    v.initial[V] = 5;
    v.initial[Theta] = v.heading;
    v.exit = {40 * ux + 12 * uy, 40 * uy - 12 * ux};
    v.desired = {410 * ux + 12 * uy, 410 * uy - 12 * ux};
    s.vehicles.push_back(v);
  }
  for (auto &v : s.vehicles) {
    auto c = center(v, v.initial);
    v.initial[Xc] = c[0];
    v.initial[Yc] = c[1];
  }
  return s;
}
double minimum_clearance(const Scenario &s, const Fleet &fleet) {
  double clearance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < fleet.size(); ++i)
    for (size_t k = 0; k < fleet[i].size(); ++k) {
      auto c = center(s.vehicles[i], fleet[i][k]);
      double r = s.vehicles[i].radius();
      clearance = std::min(
          {clearance, c[0] - s.workspace[0] - r, s.workspace[1] - c[0] - r,
           c[1] - s.workspace[2] - r, s.workspace[3] - c[1] - r});
      for (auto b : s.obstacles)
        clearance = std::min(
            clearance, std::hypot(std::max({b[0] - c[0], c[0] - b[1], 0.}),
                                  std::max({b[2] - c[1], c[1] - b[3], 0.})) -
                           r);
      for (size_t j = i + 1; j < fleet.size(); ++j) {
        auto d = center(s.vehicles[j], fleet[j][k]);
        clearance = std::min(clearance, std::hypot(c[0] - d[0], c[1] - d[1]) -
                                            r - s.vehicles[j].radius());
      }
    }
  return clearance;
}
} // namespace aim
