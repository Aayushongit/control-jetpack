function command = jh_allocate_commands(forceBodyTarget, torqueBodyTarget, params)
deltaWrench = [
    forceBodyTarget - [0.0; 0.0; params.weight];
    torqueBodyTarget
];

deltaControl = pinv(params.controlEffectiveness) * deltaWrench;

command.thrust = params.hover.thrust;
command.pitch = zeros(4, 1);
command.roll = zeros(4, 1);

for idx = 1:4
    base = 3 * (idx - 1);
    command.thrust(idx) = command.thrust(idx) + deltaControl(base + 1);
    command.pitch(idx) = command.pitch(idx) + deltaControl(base + 2);
    command.roll(idx) = command.roll(idx) + deltaControl(base + 3);
end

command = jh_apply_command_limits(command, params);
end
