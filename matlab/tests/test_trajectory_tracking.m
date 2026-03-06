function result = test_trajectory_tracking(params)
scenario.name = 'Trajectory Tracking';
scenario.dt = params.simulation.dt;
scenario.duration = 18.0;
scenario.enableGround = false;
scenario.x0 = [0.0; 0.0; 1.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
scenario.referenceFcn = @figure_eight_reference;

simulation = jh_simulate_scenario(params, scenario);

trackingError = vecnorm(simulation.state(1:3, :) - simulation.reference.position, 2, 1);
rmsPositionError = sqrt(mean(trackingError .^ 2));
peakPositionError = max(trackingError);
finalPositionError = trackingError(end);
maxTiltDeg = max(max(abs(simulation.state(7:8, :)))) * 180.0 / pi;
peakSpeed = max(vecnorm(simulation.state(4:6, :), 2, 1));

score = max(0.0, 1.0 ...
    - 1.40 * rmsPositionError ...
    - 0.35 * peakPositionError ...
    - 0.60 * finalPositionError ...
    - 0.02 * maxTiltDeg ...
    - 0.03 * max(peakSpeed - 2.0, 0.0));

result.name = scenario.name;
result.simulation = simulation;
result.metrics.rmsPositionError = rmsPositionError;
result.metrics.peakPositionError = peakPositionError;
result.metrics.finalPositionError = finalPositionError;
result.metrics.maxTiltDeg = maxTiltDeg;
result.metrics.peakSpeed = peakSpeed;
result.pass = rmsPositionError < 0.30 && peakPositionError < 0.70 && finalPositionError < 0.20 && maxTiltDeg < 20.0;
result.score = score;
result.summary = sprintf('rms err %.3f m, peak err %.3f m, peak speed %.2f m/s', rmsPositionError, peakPositionError, peakSpeed);
end

function reference = figure_eight_reference(time, ~, ~)
startTime = 1.0;
rampDuration = 2.5;
omega = 0.55;
amplitudeX = 0.90;
amplitudeY = 0.45;
yawAmplitude = 8.0 * pi / 180.0;

tau = max(time - startTime, 0.0);
[alpha, alphaDot, alphaDDot] = smoothstep_profile(tau, rampDuration);

xBase = amplitudeX * sin(omega * tau);
yBase = amplitudeY * sin(2.0 * omega * tau);
xDotBase = amplitudeX * omega * cos(omega * tau);
yDotBase = 2.0 * amplitudeY * omega * cos(2.0 * omega * tau);
xDDotBase = -amplitudeX * omega * omega * sin(omega * tau);
yDDotBase = -4.0 * amplitudeY * omega * omega * sin(2.0 * omega * tau);
yawBase = yawAmplitude * sin(omega * tau);
yawDotBase = yawAmplitude * omega * cos(omega * tau);

position = [alpha * xBase; alpha * yBase; 1.0];
velocity = [alphaDot * xBase + alpha * xDotBase; alphaDot * yBase + alpha * yDotBase; 0.0];
acceleration = [ ...
    alphaDDot * xBase + 2.0 * alphaDot * xDotBase + alpha * xDDotBase; ...
    alphaDDot * yBase + 2.0 * alphaDot * yDotBase + alpha * yDDotBase; ...
    0.0];
yaw = alpha * yawBase;
yawRate = alphaDot * yawBase + alpha * yawDotBase;

reference = jh_reference_struct( ...
    position, ...
    velocity, ...
    acceleration, ...
    [0.0; 0.0; yaw], ...
    [0.0; 0.0; yawRate]);
end

function [value, firstDerivative, secondDerivative] = smoothstep_profile(time, duration)
if time <= 0.0
    value = 0.0;
    firstDerivative = 0.0;
    secondDerivative = 0.0;
    return;
end

if time >= duration
    value = 1.0;
    firstDerivative = 0.0;
    secondDerivative = 0.0;
    return;
end

u = time / duration;
value = u * u * (3.0 - 2.0 * u);
firstDerivative = 6.0 * u * (1.0 - u) / duration;
secondDerivative = (6.0 - 12.0 * u) / (duration * duration);
end
