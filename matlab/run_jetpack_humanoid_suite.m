function results = run_jetpack_humanoid_suite(makePlots)
if nargin < 1
    makePlots = true;
end

rootDir = fileparts(mfilename('fullpath'));
srcDir = fullfile(rootDir, 'src');
testsDir = fullfile(rootDir, 'tests');
resultsDir = fullfile(rootDir, 'results');

addpath(srcDir);
addpath(testsDir);

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

params = jh_params();

results.metadata.generated_at = datestr(now, 30);
results.metadata.root_dir = rootDir;
results.metadata.source_model = params.sourceModel;
results.metadata.source_asset_dirs = params.sourceAssetDirs;
results.metadata.simplified_urdf = fullfile(rootDir, 'urdf_models', 'jetpack_humanoid_simplified.urdf');
results.metadata.testOrder = {
    'hover'
    'takeoff'
    'landing'
    'maneuverability'
    'yaw'
    'disturbance'
    'trajectory'
    'payloadSensitivity'
    'actuatorStress'
    'failureCase'
    'gainSweep'
};
results.capability = jh_analyze_capability(params);

results.hover = test_hover(params);
results.takeoff = test_takeoff(params);
results.landing = test_landing(params);
results.maneuverability = test_maneuverability(params);
results.yaw = test_yaw_authority(params);
results.disturbance = test_disturbance_rejection(params);
results.trajectory = test_trajectory_tracking(params);
results.payloadSensitivity = test_payload_sensitivity(params);
results.actuatorStress = test_actuator_limits(params);
results.failureCase = test_failure_case(params);
results.gainSweep = test_gain_sweep(params);

jh_print_report(results);
if makePlots
    jh_plot_results(results);
end

save(fullfile(resultsDir, 'jetpack_humanoid_suite_results.mat'), 'results');
end
