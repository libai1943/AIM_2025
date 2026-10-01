#include "aim/planner.hpp"
#include <algorithm>
#include <casadi/casadi.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
namespace aim {
using casadi::DM;
using casadi::SX;
Solution optimize(const Scenario &scene, Fleet reference,
                  const std::string &linear_solver) {
  const auto &p = scene.parameters;
  const int nv = int(reference.size()), state_count = nv * p.nodes * Dimension;
  double weight = p.penalty;
  Solution solution;
  for (int iteration = 1; iteration <= p.max_iterations; ++iteration) {
    auto corridors = build_corridors(scene, reference);
    int slack_count = int(corridors.relative.size()) * p.nodes;
    SX z = SX::sym("expanded_state_and_slack", state_count + slack_count),
       normal = 0, kinematic = 0, slack = 0;
    auto index = [&](int i, int k, int c) {
      return (i * p.nodes + k) * Dimension + c;
    };
    auto q = [&](int i, int k, int c) { return z(index(i, k, c)); };
    std::vector<double> lower(state_count + slack_count, -1e20),
        upper(state_count + slack_count, 1e20),
        guess(state_count + slack_count, 0), gl, gu;
    std::vector<SX> expressions, defects;
    auto add = [&](SX e, double l, double u) {
      expressions.push_back(e);
      gl.push_back(l);
      gu.push_back(u);
    };
    auto soften = [&](SX defect) {
      defects.push_back(defect);
      kinematic += p.dt() * pow(defect, 2);
    };
    for (int i = 0; i < nv; ++i) {
      const auto &vehicle = scene.vehicles[i];
      for (int k = 0; k < p.nodes; ++k) {
        for (int c = 0; c < Dimension; ++c)
          guess[index(i, k, c)] = reference[i][k][c];
        lower[index(i, k, V)] = p.vmin;
        upper[index(i, k, V)] = p.vmax;
        for (int c : {A, Phi, Omega}) {
          double limit = c == A ? p.amax : c == Phi ? p.phimax : p.omegamax;
          lower[index(i, k, c)] = -limit;
          upper[index(i, k, c)] = limit;
        }
        auto box = corridors.absolute[i][k];
        lower[index(i, k, Xc)] = box[0];
        upper[index(i, k, Xc)] = box[1];
        lower[index(i, k, Yc)] = box[2];
        upper[index(i, k, Yc)] = box[3];
        soften(q(i, k, Xc) - q(i, k, X) -
               vehicle.center_offset() * cos(q(i, k, Theta)));
        soften(q(i, k, Yc) - q(i, k, Y) -
               vehicle.center_offset() * sin(q(i, k, Theta)));
        if (k < p.nodes - 1) {
          soften((q(i, k + 1, X) - q(i, k, X)) / p.dt() -
                 q(i, k, V) * cos(q(i, k, Theta)));
          soften((q(i, k + 1, Y) - q(i, k, Y)) / p.dt() -
                 q(i, k, V) * sin(q(i, k, Theta)));
          soften((q(i, k + 1, V) - q(i, k, V)) / p.dt() - q(i, k, A));
          soften((q(i, k + 1, Phi) - q(i, k, Phi)) / p.dt() - q(i, k, Omega));
          soften((q(i, k + 1, Theta) - q(i, k, Theta)) / p.dt() -
                 q(i, k, V) * tan(q(i, k, Phi)) / vehicle.LW);
          normal += p.smoothness * p.dt() *
                    (pow(q(i, k, A), 2) + pow(q(i, k, Omega), 2));
        }
      }
      for (int c = 0; c < 7; ++c)
        lower[index(i, 0, c)] = upper[index(i, 0, c)] = guess[index(i, 0, c)] =
            vehicle.initial[c];
      int last = p.nodes - 1;
      for (int c : {V, A, Phi, Omega, Theta}) {
        double value = c == V       ? vehicle.exit_speed
                       : c == Theta ? vehicle.heading
                                    : 0;
        lower[index(i, last, c)] = upper[index(i, last, c)] =
            guess[index(i, last, c)] = value;
      }
      double ux = cos(vehicle.heading), uy = sin(vehicle.heading);
      if (std::abs(ux) < 1e-12) ux = 0;
      if (std::abs(uy) < 1e-12) uy = 0;
      // Endpoint lies beyond the exit on its outbound ray, Eq.(6); it is not
      // fixed.
      add((q(i, last, X) - vehicle.exit[0]) * ux +
              (q(i, last, Y) - vehicle.exit[1]) * uy,
          0, 1e20);
      add(-(q(i, last, X) - vehicle.exit[0]) * uy +
              (q(i, last, Y) - vehicle.exit[1]) * ux,
          0, 0);
      normal += pow(q(i, last, X) - vehicle.desired[0], 2) +
                pow(q(i, last, Y) - vehicle.desired[1], 2);
    }
    for (size_t pair = 0; pair < corridors.relative.size(); ++pair) {
      const auto &corridor = corridors.relative[pair];
      for (int k = 0; k < p.nodes; ++k) {
        int d = state_count + int(pair) * p.nodes + k;
        lower[d] = 0;
        SX delta = z(d);
        slack += p.slack_weight * delta;
        auto b = corridor.boxes[k];
        SX dx = q(corridor.first, k, Xc) - q(corridor.second, k, Xc),
           dy = q(corridor.first, k, Yc) - q(corridor.second, k, Yc);
        add(dx + delta, b[0], 1e20);
        add(dx - delta, -1e20, b[1]);
        add(dy + delta, b[2], 1e20);
        add(dy - delta, -1e20, b[3]);
        guess[d] = std::max({0.,
                             b[0] - double(reference[corridor.first][k][Xc] -
                                           reference[corridor.second][k][Xc]),
                             double(reference[corridor.first][k][Xc] -
                                    reference[corridor.second][k][Xc]) -
                                 b[1],
                             b[2] - double(reference[corridor.first][k][Yc] -
                                           reference[corridor.second][k][Yc]),
                             double(reference[corridor.first][k][Yc] -
                                    reference[corridor.second][k][Yc]) -
                                 b[3]});
      }
    }
    // Constant objective scaling preserves the minimizer and improves
    // conditioning.
    SX objective = kinematic + (normal + slack) / weight;
    casadi::Dict options;
    options["print_time"] = false;
    options["ipopt.print_level"] = 0;
    options["ipopt.sb"] = "yes";
    options["ipopt.tol"] = weight >= 1e8 ? 1e-12 : 1e-10;
    options["ipopt.acceptable_iter"] = 0;
    options["ipopt.max_iter"] = 3000;
    options["ipopt.max_cpu_time"] = 180.;
    options["ipopt.linear_solver"] = linear_solver;
    options["ipopt.mu_strategy"] = "adaptive";
    options["ipopt.ma27_pivtol"] = 1e-6;
    options["ipopt.bound_relax_factor"] = 0.;
    options["ipopt.nlp_scaling_method"] = "none";
    if (const char *hsl = std::getenv("HSL_LIBRARY"))
      options["ipopt.hsllib"] = hsl;
    SX residual_vector = SX::vertcat(defects),
       jacobian = SX::jacobian(residual_vector, z);
    SX hessian = 2 * p.dt() * SX::mtimes(jacobian.T(), jacobian) +
                 SX::hessian(normal, z) / weight;
    SX multiplier = SX::sym("objective_multiplier"),
       constraint_multipliers =
           SX::sym("constraint_multipliers", int(expressions.size()));
    // Gauss-Newton curvature for the sum of squared nonlinear defects. The
    // constraints are linear, so their Lagrangian Hessian contribution is zero.
    options["hess_lag"] = casadi::Function("gauss_newton",
                                           {z, SX::sym("unused_parameters", 0),
                                            multiplier, constraint_multipliers},
                                           {SX::triu(multiplier * hessian)});
    auto solver = casadi::nlpsol(
        "liom", "ipopt",
        casadi::SXDict{
            {"x", z}, {"f", objective}, {"g", SX::vertcat(expressions)}},
        options);
    auto answer = solver(casadi::DMDict{{"x0", guess},
                                        {"lbx", lower},
                                        {"ubx", upper},
                                        {"lbg", gl},
                                        {"ubg", gu}});
    if (!bool(solver.stats().at("success")))
      throw std::runtime_error("LIOM NLP failed: " +
                               solver.stats().at("return_status").to_string());
    auto values = answer.at("x").nonzeros();
    // For fixed states the optimal slack is available analytically. Remove
    // unnecessary interior-point barrier slack without changing any trajectory.
    for (size_t pair = 0; pair < corridors.relative.size(); ++pair) {
      const auto &c = corridors.relative[pair];
      for (int k = 0; k < p.nodes; ++k) {
        auto b = c.boxes[k];
        double dx = values[index(c.first, k, Xc)] -
                    values[index(c.second, k, Xc)],
               dy = values[index(c.first, k, Yc)] -
                    values[index(c.second, k, Yc)];
        values[state_count + int(pair) * p.nodes + k] =
            std::max({0., b[0] - dx, dx - b[1], b[2] - dy, dy - b[3]});
      }
    }
    answer["x"] = DM(values);
    casadi::Function hard_constraints("hard_constraints", {z},
                                      {SX::vertcat(expressions)});
    auto constraints =
        hard_constraints(std::vector<DM>{answer.at("x")})[0].nonzeros();
    double linear_violation = 0;
    for (size_t j = 0; j < values.size(); ++j)
      linear_violation = std::max(
          {linear_violation, lower[j] - values[j], values[j] - upper[j]});
    for (size_t j = 0; j < constraints.size(); ++j)
      linear_violation = std::max(
          {linear_violation, gl[j] - constraints[j], constraints[j] - gu[j]});
    if (linear_violation > 1e-6)
      throw std::runtime_error("Hard linear constraint violation");
    casadi::Function terms(
        "terms", {z},
        {normal, weight * kinematic, slack, SX::vertcat(defects)});
    auto cost = terms(std::vector<DM>{answer.at("x")});
    double jnormal = double(cost[0]), jkin = double(cost[1]),
           jslack = std::max(0., double(cost[2]));
    solution.max_defect = 0;
    for (double d : cost[3].nonzeros())
      solution.max_defect = std::max(solution.max_defect, std::abs(d));
    for (int i = 0; i < nv; ++i)
      for (int k = 0; k < p.nodes; ++k)
        for (int c = 0; c < Dimension; ++c)
          reference[i][k][c] = values[index(i, k, c)];
    solution.minimum_clearance = minimum_clearance(scene, reference);
    solution.iterations.push_back(
        {iteration, weight, jnormal, jkin, jslack, jkin + jslack});
    std::cout << "LIOM " << iteration << ": normal=" << jnormal
              << ", Jkin=" << jkin << ", Jslack=" << jslack
              << ", clearance=" << solution.minimum_clearance
              << ", penalty=" << weight << std::endl;
    if (jkin + jslack <= p.tolerance && solution.minimum_clearance >= -1e-6) {
      solution.fleet = std::move(reference);
      return solution;
    }
    // External-penalty continuation implements the large-penalty limit
    // discussed in the appendix. Initial weight is Table II's 200; this is a
    // documented numerical choice for this independently implemented
    // demonstration.
    weight *= 200;
  }
  throw std::runtime_error("LIOM did not meet the weighted infeasibility "
                           "threshold within the iteration budget");
}
} // namespace aim
