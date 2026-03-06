function derivative = jh_state_derivative(state, command, params)
position = state(1:3);
velocity = state(4:6);
attitude = state(7:9);
omega = state(10:12);

rotation = jh_rotation_matrix(attitude);
[forceBody, torqueBody] = jh_wrench_from_commands(params, command);
forceWorld = rotation * forceBody;

linearAcceleration = forceWorld / params.mass - [0.0; 0.0; params.gravity] - params.drag.linear .* velocity / params.mass;
angularAcceleration = params.inertia \ (torqueBody - cross(omega, params.inertia * omega) - params.drag.angular .* omega);

derivative = zeros(12, 1);
derivative(1:3) = velocity;
derivative(4:6) = linearAcceleration;
derivative(7:9) = jh_euler_rate_matrix(attitude) * omega;
derivative(10:12) = angularAcceleration;

if position(3) <= 0.0 && derivative(3) < 0.0
    derivative(3) = 0.0;
end
end
