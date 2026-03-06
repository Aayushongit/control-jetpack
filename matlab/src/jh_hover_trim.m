function hover = jh_hover_trim(params)
rearArm = abs(0.5 * (params.thrusters(1).position(1) + params.thrusters(2).position(1)));
frontArm = abs(0.5 * (params.thrusters(3).position(1) + params.thrusters(4).position(1)));

frontEach = params.weight / (2.0 * (1.0 + frontArm / rearArm));
rearEach = (frontArm / rearArm) * frontEach;

hover.thrust = [rearEach; rearEach; frontEach; frontEach];
hover.pitch = zeros(4, 1);
hover.roll = zeros(4, 1);
end
