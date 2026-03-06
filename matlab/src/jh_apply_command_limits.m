function command = jh_apply_command_limits(command, params)
for idx = 1:numel(params.thrusters)
    thruster = params.thrusters(idx);
    command.thrust(idx) = min(max(command.thrust(idx), 0.0), thruster.maxThrust);
    command.pitch(idx) = min(max(command.pitch(idx), -thruster.pitchLimit), thruster.pitchLimit);
    command.roll(idx) = min(max(command.roll(idx), -thruster.rollLimit), thruster.rollLimit);
end
end
