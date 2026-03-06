function jh_print_report(results)
capability = results.capability;

fprintf('\n');
fprintf('============================================================\n');
fprintf('JETPACK HUMANOID MATLAB SUITE\n');
fprintf('============================================================\n');
fprintf('Thrust-to-weight ratio: %.3f\n', capability.thrustToWeight);
fprintf('Hover thrust fraction: %.3f\n', capability.hoverThrottleFraction);
fprintf('Max net vertical acceleration: %.3f m/s^2\n', capability.maxNetVerticalAcceleration);
fprintf('Control effectiveness rank near hover: %d\n', capability.controlRank);
fprintf('Approx pure yaw torque capacity near hover: %.3f N*m\n', capability.pureYawTorque);
fprintf('Approx lateral acceleration near hover: %.3f m/s^2\n', capability.approxMaxLateralAcceleration);
fprintf('\n');

tests = {'hover', 'takeoff', 'landing', 'maneuverability', 'yaw'};
for idx = 1:numel(tests)
    name = tests{idx};
    entry = results.(name);
    fprintf('%-16s pass=%d  score=%.3f  %s\n', entry.name, entry.pass, entry.score, entry.summary);
end

fprintf('============================================================\n');
fprintf('\n');
end
