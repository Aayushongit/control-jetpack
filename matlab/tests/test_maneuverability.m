function result = test_maneuverability(params)
scenario.name = 'Maneuverability';
scenario.dt = params.simulation.dt;
scenario.duration = 14.0;
scenario.enableGround = false;
scenario.x0 = [0.0; 0.0; 1.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
scenario.referenceFcn = @maneuver_reference;

simulation = jh_simulate_scenario(params, scenario);

target = [1.00; 0.60; 1.10];
finalPositionError = norm(simulation.state(1:3, end) - target);
maxTiltDeg = max(max(abs(simulation.state(7:8, :)))) * 180.0 / pi;
peakSpeed = max(vecnorm(simulation.state(4:6, :), 2, 1));
score = max(0.0, 1.0 - 0.35 * finalPositionError - 0.015 * maxTiltDeg - 0.04 * max(peakSpeed - 2.0, 0.0));

result.name = scenario.name;
result.simulation = simulation;
result.metrics.finalPositionError = finalPositionError;
result.metrics.maxTiltDeg = maxTiltDeg;
result.metrics.peakSpeed = peakSpeed;
result.pass = finalPositionError < 0.18 && maxTiltDeg < 18.0 && peakSpeed < 3.0;
result.score = score;
result.summary = sprintf('final xyz err %.3f m, peak speed %.2f m/s', finalPositionError, peakSpeed);
end

function reference = maneuver_reference(time, ~, ~)
alpha = jh_smoothstep((time - 1.0) / 5.0);
targetPosition = [1.00; 0.60; 1.10] .* alpha;
targetYaw = (10.0 * pi / 180.0) * jh_smoothstep((time - 3.0) / 4.0);

reference = jh_reference_struct( ...
    targetPosition, ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; targetYaw], ...
    [0.0; 0.0; 0.0]);
end
