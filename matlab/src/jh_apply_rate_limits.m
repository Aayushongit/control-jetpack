function command = jh_apply_rate_limits(command, previousCommand, dt, params)
for idx = 1:numel(params.thrusters)
    thruster = params.thrusters(idx);

    thrustDelta = command.thrust(idx) - previousCommand.thrust(idx);
    thrustStep = thruster.thrustRate * dt;
    thrustDelta = min(max(thrustDelta, -thrustStep), thrustStep);
    command.thrust(idx) = previousCommand.thrust(idx) + thrustDelta;

    pitchDelta = command.pitch(idx) - previousCommand.pitch(idx);
    angleStep = thruster.gimbalRate * dt;
    pitchDelta = min(max(pitchDelta, -angleStep), angleStep);
    command.pitch(idx) = previousCommand.pitch(idx) + pitchDelta;

    rollDelta = command.roll(idx) - previousCommand.roll(idx);
    rollDelta = min(max(rollDelta, -angleStep), angleStep);
    command.roll(idx) = previousCommand.roll(idx) + rollDelta;
end

command = jh_apply_command_limits(command, params);
end
