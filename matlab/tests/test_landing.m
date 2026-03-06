function result = test_landing(params)
scenario.name = 'Landing';
scenario.dt = params.simulation.dt;
scenario.duration = 12.0;
scenario.enableGround = true;
scenario.x0 = [
    0.00; 0.00; 1.20;
    0.00; 0.00; -0.10;
    3.0; -2.0; 4.0;
    0.00; 0.00; 0.00
] .* [1; 1; 1; 1; 1; 1; pi / 180.0; pi / 180.0; pi / 180.0; 1; 1; 1];
scenario.referenceFcn = @landing_reference;

simulation = jh_simulate_scenario(params, scenario);

finalAltitude = simulation.state(3, end);
touchdownSpeed = simulation.touchdownSpeed;
if isnan(touchdownSpeed)
    touchdownSpeed = abs(simulation.state(6, end));
end
maxTiltDeg = max(max(abs(simulation.state(7:8, :)))) * 180.0 / pi;
score = max(0.0, 1.0 - 0.6 * abs(finalAltitude) - 0.4 * touchdownSpeed - 0.02 * maxTiltDeg);

result.name = scenario.name;
result.simulation = simulation;
result.metrics.finalAltitude = finalAltitude;
result.metrics.touchdownSpeed = touchdownSpeed;
result.metrics.maxTiltDeg = maxTiltDeg;
result.pass = abs(finalAltitude) < 0.02 && touchdownSpeed < 0.85 && maxTiltDeg < 16.0;
result.score = score;
result.summary = sprintf('touchdown speed %.3f m/s, final altitude %.3f m', touchdownSpeed, finalAltitude);
end

function reference = landing_reference(time, ~, ~)
alpha = jh_smoothstep((time - 1.0) / 4.5);
zTarget = 1.20 * (1.0 - alpha);

reference = jh_reference_struct( ...
    [0.0; 0.0; zTarget], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0]);
end
