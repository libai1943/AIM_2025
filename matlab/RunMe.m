function result = RunMe()
%RUNME Chapter 10: ten-vehicle cooperative trajectory planning.
% Companion to the Chinese-language book:
%   非结构化场景自动驾驶轨迹规划技术
%   Trajectory Planning Techniques for Autonomous Driving in Unstructured
%   Environments (English rendering of the Chinese book title).
%
% Please cite BOTH references, listed in full with BibTeX in ../README.md:
% 1. B. Li et al., Sharing Traffic Priorities via Cyber-Physical-Social
%    Intelligence: A Lane-Free Autonomous Intersection Management Method
%    in Metaverse, IEEE TSMC: Systems, 53(4), 2025-2036, 2023.
%    DOI: 10.1109/TSMC.2022.3225250 (first published online in 2022).
% 2. F. Bu et al., Real-Time Cooperative Trajectory Planning for Multiple
%    CAVs at Unstructured Intersections: A Computational Optimal Control
%    Approach, IEEE T-ITS, 26(11), 18963-18977, 2025.
%    DOI: 10.1109/TITS.2025.3597472.
%
% One-time setup: add the official CasADi MATLAB package to the MATLAB path,
% or set CASADI_MATLAB_PATH. Install compatible HSL/MA27 and set HSL_LIBRARY
% to its shared-library path. No AMPL or C++ executable is used here.
% Run this function to optimize all ten vehicles and display two figures.
root = fileparts(mfilename('fullpath'));
previous = path;
cleanup = onCleanup(@()path(previous)); %#ok<NASGU>
addpath(root);
package = getenv('CASADI_MATLAB_PATH');
if ~isempty(package), addpath(package); end
assert(exist('casadiMEX','file')==3,'AIM:CasADi', ...
    'Add the official CasADi MATLAB package to the path; see README.md.');
scene = CreateScenario();
fprintf('Chapter 10 | 10 CAVs | CasADi / IPOPT / MA27\n');
timer = tic;
initial = SimulateMVSFM(scene);
initial_metrics = ValidateSolution(scene,initial,false);
fprintf('MVSFM: minimum covering-disc clearance %.6f m.\n',initial_metrics.minimum_clearance);
result = OptimizeLIOM(scene,initial);
result.planning_seconds = toc(timer);
result.initial = initial;
output = fullfile(root,'results');
ExportResults(scene,result,output);
PlotResults(scene,result,output);
fprintf('Validated 2000 configurations; clearance %.6f m; planning %.3f s.\n', ...
    result.metrics.minimum_clearance,result.planning_seconds);
end
