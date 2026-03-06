function result = test_hover(params)
scenario.name = 'Hover Hold';
scenario.dt = params.simulation.dt;
scenario.duration = 10.0;
scenario.enableGround = false;
scenario.x0 = [
    0.00; 0.00; 1.00;
    0.00; 0.00; 0.00;
    4.0; -3.0; 6.0;
    0.00; 0.00; 0.00
] .* [1; 1; 1; 1; 1; 1; pi / 180.0; pi / 180.0; pi / 180.0; 1; 1; 1];
scenario.referenceFcn = @hover_reference;

simulation = jh_simulate_scenario(params, scenario);

finalPositionError = norm(simulation.state(1:3, end) - [0.0; 0.0; 1.0]);
finalYawError = abs(jh_wrap_to_pi(simulation.state(9, end)));
maxTiltDeg = max(max(abs(simulation.state(7:8, :)))) * 180.0 / pi;
score = max(0.0, 1.0 - 0.45 * finalPositionError - 0.25 * finalYawError - 0.02 * maxTiltDeg);

result.name = scenario.name;
result.simulation = simulation;
result.metrics.finalPositionError = finalPositionError;
result.metrics.finalYawErrorDeg = finalYawError * 180.0 / pi;
result.metrics.maxTiltDeg = maxTiltDeg;
result.pass = finalPositionError < 0.08 && finalYawError < 5.0 * pi / 180.0 && maxTiltDeg < 12.0;
result.score = score;
result.summary = sprintf('final pos err %.3f m, yaw err %.2f deg', finalPositionError, result.metrics.finalYawErrorDeg);
end

function reference = hover_reference(~, ~, ~)
reference = jh_reference_struct( ...
    [0.0; 0.0; 1.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0]);
end
