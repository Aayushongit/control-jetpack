function result = test_yaw_authority(params)
scenario.name = 'Yaw Authority';
scenario.dt = params.simulation.dt;
scenario.duration = 12.0;
scenario.enableGround = false;
scenario.x0 = [0.0; 0.0; 1.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
scenario.referenceFcn = @yaw_reference;

simulation = jh_simulate_scenario(params, scenario);

targetYaw = 25.0 * pi / 180.0;
finalYawError = abs(jh_wrap_to_pi(simulation.state(9, end) - targetYaw));
positionDrift = norm(simulation.state(1:2, end));
score = max(0.0, 1.0 - 0.9 * finalYawError - 0.6 * positionDrift);

result.name = scenario.name;
result.simulation = simulation;
result.metrics.finalYawErrorDeg = finalYawError * 180.0 / pi;
result.metrics.positionDrift = positionDrift;
result.pass = finalYawError < 6.0 * pi / 180.0 && positionDrift < 0.30;
result.score = score;
result.summary = sprintf('final yaw err %.2f deg, drift %.3f m', result.metrics.finalYawErrorDeg, positionDrift);
end

function reference = yaw_reference(time, ~, ~)
targetYaw = (25.0 * pi / 180.0) * jh_smoothstep((time - 1.0) / 3.0);

reference = jh_reference_struct( ...
    [0.0; 0.0; 1.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; targetYaw], ...
    [0.0; 0.0; 0.0]);
end
