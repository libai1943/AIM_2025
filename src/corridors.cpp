#include "aim/planner.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
namespace aim {
namespace {
Box expand(Point seed, const Parameters &p,
           const std::function<bool(const Box &)> &safe) {
  Box box{seed[0], seed[0], seed[1], seed[1]};
  if (!safe(box)) {
    // A soft intermediate state can lie inside the next dilated boundary.
    // Project the seed locally; the new corridor remains strictly
    // collision-free.
    bool found = false;
    for (int ring = 1; ring <= 200 && !found; ++ring)
      for (int a = 0; a < 32; ++a) {
        double angle = 2 * pi * a / 32;
        Point trial{seed[0] + ring * .05 * cos(angle),
                    seed[1] + ring * .05 * sin(angle)};
        Box b{trial[0], trial[0], trial[1], trial[1]};
        if (safe(b)) {
          box = b;
          found = true;
          break;
        }
      }
    if (!found)
      throw std::runtime_error(
          "No collision-free corridor seed in the local neighborhood");
  }
  std::array<double, 4> growth{};
  std::array<bool, 4> done{};
  int remaining = 4;
  // Up, down, left, right as in Algorithm 1.
  while (remaining)
    for (int side : {3, 2, 0, 1})
      if (!done[side]) {
        Box trial = box;
        trial[side] += (side % 2 ? 1 : -1) * p.box_step;
        if (growth[side] + p.box_step > p.box_limit || !safe(trial)) {
          done[side] = true;
          --remaining;
        } else {
          box = trial;
          growth[side] += p.box_step;
        }
      }
  return box;
}
double rectangle_distance(const Box &a, const Box &b) {
  return hypot(std::max({a[0] - b[1], b[0] - a[1], 0.}),
               std::max({a[2] - b[3], b[2] - a[3], 0.}));
}
} // namespace
Corridors build_corridors(const Scenario &scene, const Fleet &reference) {
  Corridors result;
  const auto &p = scene.parameters;
  result.absolute.resize(reference.size());
  for (size_t i = 0; i < reference.size(); ++i) {
    double r = scene.vehicles[i].radius();
    for (auto q : reference[i]) {
      auto safe = [&](const Box &b) {
        if (b[0] < scene.workspace[0] + r || b[1] > scene.workspace[1] - r ||
            b[2] < scene.workspace[2] + r || b[3] > scene.workspace[3] - r)
          return false;
        for (auto o : scene.obstacles)
          if (rectangle_distance(b, o) < r + 1e-8)
            return false;
        return true;
      };
      result.absolute[i].push_back(expand({q[Xc], q[Yc]}, p, safe));
    }
  }
  for (size_t i = 0; i < reference.size(); ++i)
    for (size_t j = i + 1; j < reference.size(); ++j) {
      double r = scene.vehicles[i].radius() + scene.vehicles[j].radius();
      bool relevant = false;
      // Detect overlap of physical swept-disc boxes, not just zero-radius
      // centers.
      for (int k = 0; k < p.nodes; ++k)
        if (rectangle_distance(result.absolute[i][k], result.absolute[j][k]) <=
            r) {
          relevant = true;
          break;
        }
      if (!relevant)
        continue;
      PairCorridor pair{int(i), int(j), {}};
      auto safe = [&](const Box &b) {
        return hypot(std::max({b[0], -b[1], 0.}),
                     std::max({b[2], -b[3], 0.})) >= r + 1e-8;
      };
      for (int k = 0; k < p.nodes; ++k)
        pair.boxes.push_back(expand({reference[i][k][Xc] - reference[j][k][Xc],
                                     reference[i][k][Yc] - reference[j][k][Yc]},
                                    p, safe));
      result.relative.push_back(std::move(pair));
    }
  return result;
}
} // namespace aim
