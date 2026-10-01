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
  Vehicle east;
  east.initial[X] = -100;
  east.initial[Y] = -8;
  east.initial[V] = 5;
  east.heading = 0;
  east.exit = {40, -8};
  east.desired = {310, -8};
  Vehicle north;
  north.LW = 5.3;
  north.LF = 1.5;
  north.LR = 1.6;
  north.LB = 3.2;
  north.initial[X] = 8;
  north.initial[Y] = -130;
  north.initial[V] = 5;
  north.initial[Theta] = pi / 2;
  north.heading = pi / 2;
  north.exit = {8, 40};
  north.desired = {8, 280};
  s.vehicles = {east, north};
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
