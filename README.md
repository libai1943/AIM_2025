# Cooperative Trajectory Planning at Unstructured Intersections

**C++ / CasADi companion example for Chapter 10 of the Chinese-language book《非结构化场景自动驾驶轨迹规划技术》** (*Trajectory Planning Techniques for Autonomous Driving in Unstructured Environments*, English rendering of the Chinese title).

This repository implements a numerical example of the computational optimal-control framework in **Bu et al., IEEE T-ITS, 2025**. Two vehicles with different dimensions traverse a lane-free intersection with unequal corner boundaries. The program generates an initial guess, constructs absolute and relative safe travel corridors, and solves cooperative trajectories using LIOM.

The executable writes numerical results and convergence diagnostics. It requires neither MATLAB nor AMPL and contains no Python implementation or visualization component.

## Method

```text
Vehicle geometry + boundary map + initial states + exit rays
                            |
             MVSFM forward-simulation initial guess
                            |
          STCs for boundaries + RSTCs for vehicle pairs
                            |
             Linear corridor/boundary constraints
              + nonlinear kinematic penalty cost
                            |
                    CasADi / IPOPT solve
                            |
           Rebuild corridors and check infeasibility
                            |
               Validated trajectories and CSVs
```

1. **MVSFM:** guiding-point attraction, exponential vehicle/boundary repulsion, steering saturation, and synchronous bicycle-model simulation.
2. **STCs:** grow axis-aligned boxes for vehicle disc centers inside dilated free space.
3. **RSTCs:** grow boxes in relative-position space outside the pairwise collision disc. Nonnegative slack variables resolve temporary corridor conflicts.
4. **LIOM:** soften the differential and disc-center equations into squared penalties. All constraints of each intermediate NLP are linear.
5. **Acceptance:** require `Jkinematics + Jslack <= 1e-6`, then check clearance using centers recomputed from the vehicle states.

This is an independently organized, single-cluster example. It does not reproduce the paper's 1,000-vehicle experiment, reservation platform, HIL system, or original experimental map.

## Install and run

### Requirements

- A **C++17 compiler** and **CMake 3.16+**.
- **CasADi**, including its C++ headers, CMake package, link library, and **IPOPT plugin**.
- The shared-library runtime supplied with that CasADi build.

CasADi supplies symbolic expressions and derivatives; IPOPT solves the nonlinear objective under the linear constraints. See the [official installation instructions](https://github.com/casadi/casadi/wiki/InstallationInstructions), [downloads](https://web.casadi.org/get/), and [documentation](https://web.casadi.org/docs/).

Use a CasADi SDK compatible with your compiler. In particular, do not mix a MinGW C++ library with an MSVC executable. The SDK must contain `casadi/casadi.hpp`, the link library, `casadi-config.cmake`, and the IPOPT plugin; a single DLL is insufficient.

### Linux

With a compatible CasADi SDK installed:

```bash
git clone https://github.com/libai1943/AIM_2025.git
cd AIM_2025
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -Dcasadi_DIR=/absolute/path/to/casadi/cmake
cmake --build build -j
export LD_LIBRARY_PATH=/absolute/path/to/casadi:$LD_LIBRARY_PATH
./build/aim_demo --output results
```

For system-style installations, libraries may be under `<prefix>/lib` and the CMake package under `<prefix>/lib/cmake/casadi`; use your actual installation paths.

To build CasADi itself against system IPOPT on Debian/Ubuntu:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git pkg-config coinor-libipopt-dev
git clone --branch 3.7.2 --depth 1 https://github.com/casadi/casadi.git casadi-source
cmake -S casadi-source -B casadi-build \
  -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON -DWITH_PYTHON=OFF \
  -DCMAKE_INSTALL_PREFIX=$PWD/casadi-install
cmake --build casadi-build -j
cmake --install casadi-build
```

Then configure this repository with `-DCMAKE_PREFIX_PATH=/absolute/path/to/casadi-install`. Building the CasADi core without `WITH_IPOPT=ON` does not provide the required solver plugin. Package availability and library layouts depend on the distribution.

### Windows / MinGW-w64

Install CMake, MinGW-w64, and a compatible CasADi SDK. In PowerShell:

```powershell
$env:PATH = "C:\path\to\casadi;C:\path\to\mingw64\bin;" + $env:PATH
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -Dcasadi_DIR=C:/path/to/casadi/cmake
cmake --build build
.\build\aim_demo.exe --output results
```

MSVC requires an MSVC-compatible SDK; its multi-configuration generator normally places the executable under `build/Release/`.

### Optional HSL

The default IPOPT linear solver is **MUMPS**. HSL is optional and not bundled. With a separately installed, compatible HSL library, set `HSL_LIBRARY` to its full path and run:

```bash
./build/aim_demo --linear-solver ma57 --output results
```

The paper used MA27. Keep the chosen HSL library's runtime dependencies visible to the dynamic loader.

## Numerical results

A successful run returns exit code 0 and creates:

| File | Contents |
|---|---|
| `results/trajectories.csv` | `vehicle,t,x,y,v,a,phi,omega,theta,xc,yc` at each configuration |
| `results/iterations.csv` | Iteration, penalty weight, normal cost, kinematic cost, slack cost, weighted infeasibility |

Lengths are in metres, time in seconds, and angles in radians. `(x,y)` denotes the rear-axle midpoint; `(xc,yc)` is the auxiliary disc center. Each vehicle has 200 configurations, including both endpoints of the 40-second horizon. Failed optimization or validation returns a nonzero exit code.

## Mapping to the paper

| Paper component | Implementation |
|---|---|
| Bicycle dynamics and limits, Eqs. (2)–(3) | `simulation.cpp`, `optimization.cpp` |
| Initial states and outbound endpoint rays, Eqs. (5)–(7) | `scenario.cpp`, `optimization.cpp` |
| Smoothness and desired-target objective, Eq. (8) | `optimization.cpp` |
| Disc-center geometry, Eq. (10a) | `scenario.cpp`, `optimization.cpp` |
| STC / RSTC, Algorithms 1–2 | `corridors.cpp` |
| Relative corridor slack, Eqs. (18)–(19) | `optimization.cpp` |
| MVSFM forces and steering, Eqs. (21)–(25) | `simulation.cpp` |
| LIOM penalties and acceptance, Eqs. (26)–(29), Algorithm 3 | `optimization.cpp` |

Terminal positions may advance along their outbound rays. They are **not fixed to the desired targets**, which lie beyond the distance reachable within the planning horizon.

### Parameters from Table II

| Parameter | Value |
|---|---:|
| Horizon / configurations per vehicle | 40 s / 200 |
| Maximum LIOM solves | 6 |
| Weighted infeasibility threshold | 1e-6 |
| Smoothness weight | 0.001 |
| Initial kinematic penalty | 200 |
| Attractive / repulsive weights | 0.001 / 0.6 |
| Turning coefficient | 0.6 |
| Speed bounds | [3, 10] m/s |
| Acceleration bounds | [-2, 2] m/s² |
| Steering / steering-rate limits | 0.4 rad / 0.3 rad/s |

### Explicit implementation conventions

These choices distinguish a reproducible demonstration from the original experimental package:

- **Discretization:** 200 endpoint-inclusive configurations, forward Euler with `dt = 40/199`, and left rectangular quadrature for time integrals.
- **Vehicle coverage:** use Eq. (10a)'s center and the geometric radius `0.5*hypot(LW+LF+LR, LB)`. This covers the complete rectangle; the printed subscripts in Eq. (10b) do not yield that covering radius.
- **Guiding routes:** the two straight routes are zero-control-cost solutions of their individual boundary-only guiding problems. MVSFM selects the second forward point on a 2 m route grid. Longitudinal force is projected onto the heading with unit mass; the paper does not specify that conversion or the distance-scale value. The demo uses a 0.5 m distance scale.
- **Corridors:** expansion uses 0.25 m steps and a 5 m limit per side. Potential pair selection includes the disc radii. If a soft intermediate state enters a dilated boundary, its corridor seed is moved to nearby free space before expansion.
- **Penalty continuation:** start with Table II's weight 200 and multiply by 100 after an unsuccessful outer iteration. This is a numerical implementation of the large-penalty limit discussed in the appendix; the multiplier is a demo setting, not a Table II value. Acceptance uses the current weighted penalty.
- **NLP solution:** use a Gauss–Newton Hessian approximation for squared defects and constant objective scaling. At the computed states, each slack is reduced analytically to its smallest linearly feasible value, removing unnecessary interior-point barrier slack.

The metaverse paper below provides related background on lane-free management and priority sharing. This example implements the 2025 corridor/LIOM framework; it does not implement virtual-currency trading.

## Source map

| File | Responsibility |
|---|---|
| `include/aim/planner.hpp` | State components, vehicle geometry, parameters, corridor/result types |
| `src/scenario.cpp` | Constructed intersection, heterogeneous vehicles, disc geometry, clearance calculation |
| `src/simulation.cpp` | Synchronous multi-vehicle forces and bicycle-model simulation |
| `src/corridors.cpp` | Absolute/relative box growth and potentially interacting pair selection |
| `src/optimization.cpp` | CasADi model, penalties, linear constraints, solver, LIOM loop and acceptance |
| `src/main.cpp` | Command-line entry, orchestration, CSV export, failure reporting |
| `CMakeLists.txt` | Build and numerical smoke test |
| `CITATION.bib` | Both required references |

This is a planning example, not a low-level controller. Validation applies to the discrete configurations and Euler transcription; no continuous-time collision certificate is claimed.

### Verified example

The default example was compiled with GCC 16.2 and CasADi 3.7.2 on Windows and executed with IPOPT/MUMPS. Independent recomputation from the exported CSVs checked all 400 configurations, endpoint rays, control bounds, forward-Euler defects, disc geometry, and pairwise/boundary clearance. The run completed six LIOM solves with weighted infeasibility approximately `6.41e-7` and minimum covering-disc clearance approximately `5.15 m`. These are results of this constructed example, not the paper's benchmark statistics. Linux build instructions are provided; Linux execution has not been verified here.

## Citation

Please cite **both** papers when using this companion implementation:

> Fan Bu, Bai Li, Peng Chen, Guizhen Yu, and Yunpeng Wang, “Real-Time Cooperative Trajectory Planning for Multiple CAVs at Unstructured Intersections: A Computational Optimal Control Approach,” *IEEE Transactions on Intelligent Transportation Systems*, 26(11), 18963–18977, 2025. [DOI: 10.1109/TITS.2025.3597472](https://doi.org/10.1109/TITS.2025.3597472).

> Bai Li, Dongpu Cao, Shiqi Tang, Tantan Zhang, Hairong Dong, Yaonan Wang, and Fei-Yue Wang, “Sharing Traffic Priorities via Cyber–Physical–Social Intelligence: A Lane-Free Autonomous Intersection Management Method in Metaverse,” *IEEE Transactions on Systems, Man, and Cybernetics: Systems*, 53(4), 2025–2036, 2023 (online in 2022). [DOI: 10.1109/TSMC.2022.3225250](https://doi.org/10.1109/TSMC.2022.3225250).

BibTeX entries are in [CITATION.bib](CITATION.bib). Also follow [CasADi's citation guidance](https://web.casadi.org/docs/#citing-casadi) when publishing work using that numerical framework.
