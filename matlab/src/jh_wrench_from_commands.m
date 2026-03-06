function [forceBody, torqueBody] = jh_wrench_from_commands(params, command)
count = numel(params.thrusters);
directions = jh_thruster_directions(command.pitch, command.roll);

forceBody = zeros(3, 1);
torqueBody = zeros(3, 1);

for idx = 1:count
    thrustVector = command.thrust(idx) * directions(:, idx);
    forceBody = forceBody + thrustVector;
    torqueBody = torqueBody + cross(params.thrusters(idx).position, thrustVector);
end
end
