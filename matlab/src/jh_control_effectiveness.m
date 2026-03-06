function B = jh_control_effectiveness(params, hover)
B = zeros(6, 12);

for idx = 1:numel(params.thrusters)
    column = 3 * (idx - 1) + 1;
    r = params.thrusters(idx).position - params.centerOfMass;

    thrustForce = [0.0; 0.0; 1.0];
    pitchForce = hover.thrust(idx) * [1.0; 0.0; 0.0];
    rollForce = hover.thrust(idx) * [0.0; -1.0; 0.0];

    B(:, column) = [thrustForce; cross(r, thrustForce)];
    B(:, column + 1) = [pitchForce; cross(r, pitchForce)];
    B(:, column + 2) = [rollForce; cross(r, rollForce)];
end
end
