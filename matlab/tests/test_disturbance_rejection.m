function result = test_disturbance_rejection(params)
scenario.name = 'Disturbance Rejection';
scenario.dt = params.simulation.dt;
scenario.duration = 10.0;
scenario.enableGround = false;
scenario.x0 = [0.0; 0.0; 1.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
scenario.referenceFcn = @hover_reference;
scenario.disturbanceFcn = @disturbance_pulse;

simulation = jh_simulate_scenario(params, scenario);

pulseEnd = 2.25;
time = simulation.time;
positionError = vecnorm(simulation.state(1:3, :) - simulation.reference.position, 2, 1);
yawError = abs(jh_wrap_to_pi(simulation.state(9, :) - simulation.reference.attitude(3, :)));
postPulseMask = time >= pulseEnd;
recoveryPositionError = positionError(find(time >= pulseEnd + 3.0, 1));
recoveryTime = jh_settling_time(time(postPulseMask) - pulseEnd, ...
    [positionError(postPulseMask); yawError(postPulseMask)], [0.12; 4.0 * pi / 180.0]);
peakPositionError = max(positionError(postPulseMask));
finalYawError = yawError(end);
maxTiltDeg = max(max(abs(simulation.state(7:8, :)))) * 180.0 / pi;

recoveryTimeForScore = recoveryTime;
if isnan(recoveryTimeForScore)
    recoveryTimeForScore = scenario.duration;
end

score = max(0.0, 1.0 ...
    - 0.50 * peakPositionError ...
    - 0.70 * recoveryPositionError ...
    - 0.08 * recoveryTimeForScore ...
    - 0.02 * maxTiltDeg ...
    - 0.05 * finalYawError * 180.0 / pi);

result.name = scenario.name;
result.simulation = simulation;
result.metrics.peakPositionError = peakPositionError;
result.metrics.recoveryPositionError = recoveryPositionError;
result.metrics.recoveryTime = recoveryTime;
result.metrics.finalYawErrorDeg = finalYawError * 180.0 / pi;
result.metrics.maxTiltDeg = maxTiltDeg;
result.pass = peakPositionError < 0.60 && recoveryPositionError < 0.15 && ~isnan(recoveryTime) && recoveryTime < 4.0 && finalYawError < 4.0 * pi / 180.0 && maxTiltDeg < 20.0;
result.score = score;
result.summary = sprintf('peak err %.3f m, recover %.2f s, final yaw %.2f deg', peakPositionError, recoveryTimeForScore, result.metrics.finalYawErrorDeg);
end

function reference = hover_reference(~, ~, ~)
reference = jh_reference_struct( ...
    [0.0; 0.0; 1.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0]);
end

function disturbance = disturbance_pulse(time, ~, ~, ~)
disturbance.forceWorld = zeros(3, 1);
disturbance.torqueBody = zeros(3, 1);

if time >= 2.0 && time <= 2.25
    disturbance.forceWorld = [160.0; -90.0; 0.0];
    disturbance.torqueBody = [0.0; 6.0; 12.0];
end
end
