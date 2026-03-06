function result = jh_simulate_scenario(params, scenario)
dt = scenario.dt;
time = 0.0:dt:scenario.duration;
stepCount = numel(time);

state = zeros(12, stepCount);
state(:, 1) = scenario.x0;

command.thrust = params.hover.thrust;
command.pitch = params.hover.pitch;
command.roll = params.hover.roll;

commands.thrust = zeros(4, stepCount);
commands.pitch = zeros(4, stepCount);
commands.roll = zeros(4, stepCount);
references.position = zeros(3, stepCount);
references.attitude = zeros(3, stepCount);
achieved.forceBody = zeros(3, stepCount);
achieved.torqueBody = zeros(3, stepCount);

touchdownDetected = false;
touchdownSpeed = NaN;

for idx = 1:stepCount
    currentTime = time(idx);
    currentState = state(:, idx);
    reference = scenario.referenceFcn(currentTime, currentState, params);
    command = jh_controller(currentTime, currentState, reference, command, dt, params);

    commands.thrust(:, idx) = command.thrust;
    commands.pitch(:, idx) = command.pitch;
    commands.roll(:, idx) = command.roll;
    references.position(:, idx) = reference.position;
    references.attitude(:, idx) = [reference.roll; reference.pitch; reference.yaw];
    [achieved.forceBody(:, idx), achieved.torqueBody(:, idx)] = jh_wrench_from_commands(params, command);

    if idx == stepCount
        continue;
    end

    k1 = jh_state_derivative(currentState, command, params);
    k2 = jh_state_derivative(currentState + 0.5 * dt * k1, command, params);
    k3 = jh_state_derivative(currentState + 0.5 * dt * k2, command, params);
    k4 = jh_state_derivative(currentState + dt * k3, command, params);
    nextState = currentState + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    if scenario.enableGround && nextState(3) <= 0.0
        if ~touchdownDetected && currentState(3) > 0.0
            touchdownDetected = true;
            touchdownSpeed = abs(nextState(6));
        end
        nextState(3) = 0.0;
        if nextState(6) < 0.0
            nextState(6) = 0.0;
        end
    end

    state(:, idx + 1) = nextState;
end

result.name = scenario.name;
result.time = time;
result.state = state;
result.command = commands;
result.reference = references;
result.achieved = achieved;
result.touchdownDetected = touchdownDetected;
result.touchdownSpeed = touchdownSpeed;
result.finalState = state(:, end);
end
