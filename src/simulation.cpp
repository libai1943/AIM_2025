#include "aim/planner.hpp"
#include <algorithm>
#include <cmath>
namespace aim {
Fleet simulate(const Scenario &scene) {
  const auto &p = scene.parameters;
  Fleet fleet(scene.vehicles.size(), Trajectory(p.nodes));
  for (size_t i = 0; i < fleet.size(); ++i)
    fleet[i][0] = scene.vehicles[i].initial;
  // Guiding routes for this example are straight, boundary-feasible OCP
  // solutions at constant speed. Their second forward point is 4 m ahead on a 2
  // m grid.
  for (int k = 0; k < p.nodes - 1; ++k) {
    std::vector<State> next(fleet.size());
    for (size_t i = 0; i < fleet.size(); ++i) {
      auto q = fleet[i][k];
      const auto &vehicle = scene.vehicles[i];
      auto c = center(vehicle, q);
      double ux = cos(vehicle.heading), uy = sin(vehicle.heading);
      double along =
          (q[X] - vehicle.initial[X]) * ux + (q[Y] - vehicle.initial[Y]) * uy;
      double target = 2 * (std::floor(along / 2) + 2);
      Point guide{vehicle.initial[X] + target * ux,
                  vehicle.initial[Y] + target * uy};
      Point force{0, 0};
      auto attract = [&](double dx, double dy) {
        double length = std::max(1e-12, hypot(dx, dy));
        force[0] += p.attractive * dx / length;
        force[1] += p.attractive * dy / length;
      };
      attract(guide[0] - q[X], guide[1] - q[Y]);
      auto repel = [&](double dx, double dy, double radius) {
        double distance = std::max(1e-9, hypot(dx, dy)),
               magnitude =
                   p.repulsive *
                   exp(std::clamp((radius - distance) / p.distance_scale, -700.,
                                  30.));
        force[0] += magnitude * dx / distance;
        force[1] += magnitude * dy / distance;
      };
      for (size_t j = 0; j < fleet.size(); ++j)
        if (j != i) {
          auto other = center(scene.vehicles[j], fleet[j][k]);
          repel(c[0] - other[0], c[1] - other[1],
                vehicle.radius() + scene.vehicles[j].radius());
        }
      for (auto b : scene.obstacles) {
        double bx = std::clamp(c[0], b[0], b[1]),
               by = std::clamp(c[1], b[2], b[3]);
        repel(c[0] - bx, c[1] - by, vehicle.radius());
      }
      double desired_phi =
          std::clamp(p.turning * wrap(atan2(force[1], force[0]) - q[Theta]),
                     -p.phimax, p.phimax);
      // Eq. (25) sets the steering angle directly in MVSFM. Steering-rate
      // consistency is recovered by LIOM, rather than adding a rate-limited
      // steering servo that is absent from the initialization method.
      q[Phi] = desired_phi;
      // The paper leaves the longitudinal force-to-acceleration scaling
      // unspecified. Unit mass and projection along the current heading are the
      // demo convention.
      q[A] = std::clamp(force[0] * cos(q[Theta]) + force[1] * sin(q[Theta]),
                        -p.amax, p.amax);
      q[A] =
          std::clamp(q[A], (p.vmin - q[V]) / p.dt(), (p.vmax - q[V]) / p.dt());
      if (k == 0) {
        q[A] = vehicle.initial[A];
        q[Phi] = vehicle.initial[Phi];
      }
      fleet[i][k] = q;
      State z = q;
      z[X] += p.dt() * q[V] * cos(q[Theta]);
      z[Y] += p.dt() * q[V] * sin(q[Theta]);
      z[Theta] += p.dt() * q[V] * tan(q[Phi]) / vehicle.LW;
      z[V] += p.dt() * q[A];
      auto cc = center(vehicle, z);
      z[Xc] = cc[0];
      z[Yc] = cc[1];
      next[i] = z;
    }
    for (size_t i = 0; i < fleet.size(); ++i)
      fleet[i][k + 1] = next[i];
  }
  for (auto &trajectory : fleet) {
    for (int k = 0; k < p.nodes - 1; ++k)
      trajectory[k][Omega] = std::clamp(
          (trajectory[k + 1][Phi] - trajectory[k][Phi]) / p.dt(),
          -p.omegamax, p.omegamax);
    trajectory.front()[Omega] = 0;
    trajectory.back()[Omega] = 0;
  }
  return fleet;
}
} // namespace aim
