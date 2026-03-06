function result = test_takeoff(params)
scenario.name = 'Takeoff';
scenario.dt = params.simulation.dt;
scenario.duration = 12.0;
scenario.enableGround = true;
scenario.x0 = zeros(12, 1);
scenario.referenceFcn = @takeoff_reference;

simulation = jh_simulate_scenario(params, scenario);

peakAltitude = max(simulation.state(3, :));
finalAltitudeError = abs(simulation.state(3, end) - 1.20);
maxTiltDeg = max(max(abs(simulation.state(7:8, :)))) * 180.0 / pi;
score = max(0.0, 1.0 - 0.30 * finalAltitudeError - 0.02 * maxTiltDeg + 0.10 * min(peakAltitude, 1.2));

result.name = scenario.name;
result.simulation = simulation;
result.metrics.peakAltitude = peakAltitude;
result.metrics.finalAltitudeError = finalAltitudeError;
result.metrics.maxTiltDeg = maxTiltDeg;
result.pass = peakAltitude > 1.0 && finalAltitudeError < 0.12 && maxTiltDeg < 15.0;
result.score = score;
result.summary = sprintf('peak alt %.2f m, final alt err %.3f m', peakAltitude, finalAltitudeError);
end

function reference = takeoff_reference(time, ~, ~)
alpha = jh_smoothstep((time - 0.5) / 3.5);
zTarget = 1.20 * alpha;

reference = jh_reference_struct( ...
    [0.0; 0.0; zTarget], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0]);
end
