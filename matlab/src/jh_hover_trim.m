function hover = jh_hover_trim(params)
count = numel(params.thrusters);
trimMatrix = zeros(3, count);

for idx = 1:count
    momentArm = params.thrusters(idx).position - params.centerOfMass;
    trimMatrix(:, idx) = [1.0; momentArm(2); -momentArm(1)];
end

hover.thrust = pinv(trimMatrix) * [params.weight; 0.0; 0.0];
hover.pitch = zeros(count, 1);
hover.roll = zeros(count, 1);
end
