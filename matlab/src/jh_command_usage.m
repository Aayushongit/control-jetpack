function usage = jh_command_usage(command, params)
count = numel(params.thrusters);
maxThrust = zeros(count, 1);
pitchLimit = zeros(count, 1);
rollLimit = zeros(count, 1);

for idx = 1:count
    thruster = params.thrusters(idx);
    maxThrust(idx) = thruster.maxThrust;
    pitchLimit(idx) = thruster.pitchLimit;
    rollLimit(idx) = thruster.rollLimit;
end

usage.thrustFraction = command.thrust ./ maxThrust;
usage.pitchFraction = abs(command.pitch) ./ pitchLimit;
usage.rollFraction = abs(command.roll) ./ rollLimit;
usage.gimbalFraction = max(usage.pitchFraction, usage.rollFraction);
usage.maxThrustFraction = max(usage.thrustFraction(:));
usage.maxPitchFraction = max(usage.pitchFraction(:));
usage.maxRollFraction = max(usage.rollFraction(:));
usage.maxGimbalFraction = max(usage.gimbalFraction(:));
usage.thrustNearLimitFraction = mean(any(usage.thrustFraction > 0.95, 1));
usage.gimbalNearLimitFraction = mean(any(usage.gimbalFraction > 0.95, 1));
end
