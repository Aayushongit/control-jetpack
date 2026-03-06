function command = jh_controller(time, state, reference, previousCommand, dt, params)
position = state(1:3);
velocity = state(4:6);
attitude = state(7:9);
omega = state(10:12);

positionError = reference.position - position;
velocityError = reference.velocity - velocity;

forceWorldTarget = params.mass * (
    reference.acceleration + ...
    params.controller.positionKp .* positionError + ...
    params.controller.positionKd .* velocityError + ...
    [0.0; 0.0; params.gravity]);

forceWorldTarget = min(max(forceWorldTarget, -params.controller.maxBodyForce), params.controller.maxBodyForce);

rotation = jh_rotation_matrix(attitude);
forceBodyTarget = rotation' * forceWorldTarget;
forceBodyTarget = min(max(forceBodyTarget, -params.controller.maxBodyForce), params.controller.maxBodyForce);

attitudeError = [
    jh_wrap_to_pi(reference.roll - attitude(1));
    jh_wrap_to_pi(reference.pitch - attitude(2));
    jh_wrap_to_pi(reference.yaw - attitude(3))
];
angularRateError = reference.angularRate - omega;

torqueBodyTarget = ...
    params.controller.attitudeKp .* attitudeError + ...
    params.controller.attitudeKd .* angularRateError;
torqueBodyTarget = min(max(torqueBodyTarget, -params.controller.maxBodyTorque), params.controller.maxBodyTorque);

command = jh_allocate_commands(forceBodyTarget, torqueBodyTarget, params);
command = jh_apply_rate_limits(command, previousCommand, dt, params);
command.reference = reference;
command.time = time;
end
