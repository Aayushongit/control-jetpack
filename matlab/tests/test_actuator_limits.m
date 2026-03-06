function result = test_actuator_limits(params)
scenario.name = 'Actuator Limit Stress';
scenario.dt = params.simulation.dt;
scenario.duration = 16.0;
scenario.enableGround = false;
scenario.x0 = [0.0; 0.0; 1.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
scenario.referenceFcn = @stress_reference;

simulation = jh_simulate_scenario(params, scenario);
usage = jh_command_usage(simulation.command, params);

finalPositionError = norm(simulation.state(1:3, end) - simulation.reference.position(:, end));
maxTiltDeg = max(max(abs(simulation.state(7:8, :)))) * 180.0 / pi;

score = max(0.0, 1.0 ...
    - 0.80 * finalPositionError ...
    - 0.02 * maxTiltDeg ...
    + 0.18 * min(usage.maxThrustFraction, 1.0) ...
    + 0.12 * min(usage.maxGimbalFraction, 1.0));

result.name = scenario.name;
result.simulation = simulation;
result.metrics.maxThrustFraction = usage.maxThrustFraction;
result.metrics.maxGimbalFraction = usage.maxGimbalFraction;
result.metrics.thrustNearLimitFraction = usage.thrustNearLimitFraction;
result.metrics.gimbalNearLimitFraction = usage.gimbalNearLimitFraction;
result.metrics.finalPositionError = finalPositionError;
result.metrics.maxTiltDeg = maxTiltDeg;
result.metrics.maxActuatorStressFraction = max(usage.maxThrustFraction, usage.maxGimbalFraction);
result.pass = result.metrics.maxActuatorStressFraction > 0.80 && finalPositionError < 0.30 && maxTiltDeg < 24.0;
result.score = score;
result.summary = sprintf('stress frac %.2f, thrust %.2f, gimbal %.2f', result.metrics.maxActuatorStressFraction, usage.maxThrustFraction, usage.maxGimbalFraction);
end

function reference = stress_reference(time, ~, ~)
waypoint0 = [0.0; 0.0; 1.0];
waypoint1 = [2.4; 1.2; 1.7];
waypoint2 = [-1.4; -1.0; 1.15];
waypoint3 = [0.0; 0.0; 1.0];
yaw0 = 0.0;
yaw1 = 25.0 * pi / 180.0;
yaw2 = -20.0 * pi / 180.0;
yaw3 = 0.0;

if time < 1.0
    position = waypoint0;
    yaw = yaw0;
elseif time < 3.0
    alpha = jh_smoothstep((time - 1.0) / 2.0);
    position = blend(waypoint0, waypoint1, alpha);
    yaw = blend(yaw0, yaw1, alpha);
elseif time < 6.0
    position = waypoint1;
    yaw = yaw1;
elseif time < 8.5
    alpha = jh_smoothstep((time - 6.0) / 2.5);
    position = blend(waypoint1, waypoint2, alpha);
    yaw = blend(yaw1, yaw2, alpha);
elseif time < 11.0
    position = waypoint2;
    yaw = yaw2;
elseif time < 13.5
    alpha = jh_smoothstep((time - 11.0) / 2.5);
    position = blend(waypoint2, waypoint3, alpha);
    yaw = blend(yaw2, yaw3, alpha);
else
    position = waypoint3;
    yaw = yaw3;
end

reference = jh_reference_struct( ...
    position, ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; 0.0], ...
    [0.0; 0.0; yaw], ...
    [0.0; 0.0; 0.0]);
end

function value = blend(startValue, endValue, alpha)
value = startValue + (endValue - startValue) * alpha;
end
