// Chapter 10 of the Chinese book: 非结构化场景自动驾驶轨迹规划技术.
// Please cite Li et al., TSMC: Systems 2023 (online 2022),
// doi:10.1109/TSMC.2022.3225250, and the following technical paper.
// Computational example for Bu et al., T-ITS 2025,
// doi:10.1109/TITS.2025.3597472.
#include "aim/planner.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
int main(int argc, char **argv) {
  try {
    std::filesystem::path output = "results";
    std::string linear_solver = "ma27";
    auto scene = aim::crossing_example();
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--output" && i + 1 < argc)
        output = std::filesystem::u8path(argv[++i]);
      else if (arg == "--linear-solver" && i + 1 < argc)
        linear_solver = argv[++i];
      else
        throw std::runtime_error(
            "Usage: aim_demo [--output DIR] [--linear-solver ma27]");
    }
    std::cout << "IPOPT linear solver: " << linear_solver << std::endl;
    auto begin = std::chrono::steady_clock::now();
    auto reference = aim::simulate(scene);
    std::cout << "MVSFM: " << reference.size() << " vehicles, "
              << scene.parameters.nodes << " nodes, clearance="
              << aim::minimum_clearance(scene, reference) << " m" << std::endl;
    auto result = aim::optimize(scene, std::move(reference), linear_solver);
    std::filesystem::create_directories(output);
    std::ofstream trajectory(output / "trajectories.csv");
    trajectory << std::setprecision(17)
               << "vehicle,t,x,y,v,a,phi,omega,theta,xc,yc\n";
    for (size_t i = 0; i < result.fleet.size(); ++i)
      for (int k = 0; k < scene.parameters.nodes; ++k) {
        trajectory << i + 1 << ',' << k * scene.parameters.dt();
        for (double value : result.fleet[i][k])
          trajectory << ',' << value;
        trajectory << '\n';
      }
    std::ofstream log(output / "iterations.csv");
    log << std::setprecision(17)
        << "iteration,penalty,Jnorm,Jkinematics,Jslack,infeasibility\n";
    for (auto row : result.iterations)
      log << row.index << ',' << row.penalty << ',' << row.normal << ','
          << row.kinematic << ',' << row.slack << ',' << row.violation << '\n';
    std::cout << "Validated cluster: minimum clearance="
              << result.minimum_clearance
              << " m, max defect=" << result.max_defect << ", planning="
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - begin)
                     .count()
              << " s\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
