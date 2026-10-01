#pragma once
#include <array>
#include <string>
#include <vector>
namespace aim {
constexpr double pi = 3.14159265358979323846;
enum Component { X, Y, V, A, Phi, Omega, Theta, Xc, Yc, Dimension };
using State = std::array<double, Dimension>;
using Point = std::array<double, 2>;
using Box = std::array<double, 4>;
using Trajectory = std::vector<State>;
using Fleet = std::vector<Trajectory>;
struct Parameters {
  int nodes = 200, max_iterations = 6;
  double horizon = 40, smoothness = .001, penalty = 200, slack_weight = 200,
         tolerance = 1e-6;
  double vmin = 3, vmax = 10, amax = 2, phimax = .4, omegamax = .3;
  double attractive = .001, repulsive = .6, turning = .6, distance_scale = .5;
  double box_step = .25, box_limit = 5;
  double dt() const { return horizon / (nodes - 1); }
};
struct Vehicle {
  double LW = 5.73, LF = 1.71, LR = 1.90, LB = 3.50;
  State initial{};
  Point exit, desired;
  double heading = 0, exit_speed = 5;
  double center_offset() const { return .5 * (LW - LR + LF); }
  double radius() const;
};
struct Scenario {
  Parameters parameters;
  std::vector<Vehicle> vehicles;
  Box workspace;
  std::vector<Box> obstacles;
};
struct PairCorridor {
  int first, second;
  std::vector<Box> boxes;
};
struct Corridors {
  std::vector<std::vector<Box>> absolute;
  std::vector<PairCorridor> relative;
};
struct Iteration {
  int index;
  double penalty, normal, kinematic, slack, violation;
};
struct Solution {
  Fleet fleet;
  std::vector<Iteration> iterations;
  double minimum_clearance = 0, max_defect = 0;
};
Scenario crossing_example();
Fleet simulate(const Scenario &);
Corridors build_corridors(const Scenario &, const Fleet &);
Solution optimize(const Scenario &, Fleet, const std::string &linear_solver);
double minimum_clearance(const Scenario &, const Fleet &);
Point center(const Vehicle &, const State &);
double wrap(double);
} // namespace aim
