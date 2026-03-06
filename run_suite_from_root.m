function results = run_suite_from_root(makePlots)
if nargin < 1
    makePlots = true;
end

projectRoot = fileparts(mfilename('fullpath'));
setup_control_jetpack_path(projectRoot);
results = run_jetpack_humanoid_suite(makePlots);
end
