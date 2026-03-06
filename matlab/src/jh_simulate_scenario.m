function result = jh_simulate_scenario(params, scenario)
dt = scenario.dt;
time = 0.0:dt:scenario.duration;
stepCount = numel(time);

if ~isfield(scenario, 'disturbanceFcn') || isempty(scenario.disturbanceFcn)
    scenario.disturbanceFcn = @zero_disturbance;
end

state = zeros(12, stepCount);
state(:, 1) = scenario.x0;

command.thrust = params.hover.thrust;
command.pitch = params.hover.pitch;
command.roll = params.hover.roll;

commands.thrust = zeros(4, stepCount);
commands.pitch = zeros(4, stepCount);
commands.roll = zeros(4, stepCount);
references.position = zeros(3, stepCount);
references.velocity = zeros(3, stepCount);
references.acceleration = zeros(3, stepCount);
references.attitude = zeros(3, stepCount);
references.angularRate = zeros(3, stepCount);
achieved.forceBody = zeros(3, stepCount);
achieved.torqueBody = zeros(3, stepCount);
disturbances.forceWorld = zeros(3, stepCount);
disturbances.torqueBody = zeros(3, stepCount);

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
    references.velocity(:, idx) = reference.velocity;
    references.acceleration(:, idx) = reference.acceleration;
    references.attitude(:, idx) = [reference.roll; reference.pitch; reference.yaw];
    references.angularRate(:, idx) = reference.angularRate;
    [achieved.forceBody(:, idx), achieved.torqueBody(:, idx)] = jh_wrench_from_commands(params, command);
    disturbance = scenario.disturbanceFcn(currentTime, currentState, params, command);
    disturbance = sanitize_disturbance(disturbance);
    disturbances.forceWorld(:, idx) = disturbance.forceWorld;
    disturbances.torqueBody(:, idx) = disturbance.torqueBody;

    if idx == stepCount
        continue;
    end

    k1 = jh_state_derivative(currentState, command, disturbance, params);
    state2 = currentState + 0.5 * dt * k1;
    disturbance2 = sanitize_disturbance(scenario.disturbanceFcn(currentTime + 0.5 * dt, state2, params, command));
    k2 = jh_state_derivative(state2, command, disturbance2, params);
    state3 = currentState + 0.5 * dt * k2;
    disturbance3 = sanitize_disturbance(scenario.disturbanceFcn(currentTime + 0.5 * dt, state3, params, command));
    k3 = jh_state_derivative(state3, command, disturbance3, params);
    state4 = currentState + dt * k3;
    disturbance4 = sanitize_disturbance(scenario.disturbanceFcn(currentTime + dt, state4, params, command));
    k4 = jh_state_derivative(state4, command, disturbance4, params);
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
result.disturbance = disturbances;
result.touchdownDetected = touchdownDetected;
result.touchdownSpeed = touchdownSpeed;
result.finalState = state(:, end);
end

function disturbance = zero_disturbance(varargin)
disturbance.forceWorld = zeros(3, 1);
disturbance.torqueBody = zeros(3, 1);
end

function disturbance = sanitize_disturbance(disturbance)
if isempty(disturbance)
    disturbance = zero_disturbance();
end

if ~isfield(disturbance, 'forceWorld')
    disturbance.forceWorld = zeros(3, 1);
end

if ~isfield(disturbance, 'torqueBody')
    disturbance.torqueBody = zeros(3, 1);
end

disturbance.forceWorld = disturbance.forceWorld(:);
disturbance.torqueBody = disturbance.torqueBody(:);
end
