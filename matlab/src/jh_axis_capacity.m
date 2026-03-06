function capacity = jh_axis_capacity(params, targetWrench)
unitControl = pinv(params.controlEffectiveness) * targetWrench;

upper = zeros(12, 1);
lower = zeros(12, 1);

for idx = 1:4
    base = 3 * (idx - 1);
    thruster = params.thrusters(idx);
    upper(base + 1) = thruster.maxThrust - params.hover.thrust(idx);
    lower(base + 1) = -params.hover.thrust(idx);
    upper(base + 2) = thruster.pitchLimit;
    lower(base + 2) = -thruster.pitchLimit;
    upper(base + 3) = thruster.rollLimit;
    lower(base + 3) = -thruster.rollLimit;
end

limits = inf(12, 1);
for idx = 1:12
    if unitControl(idx) > 1.0e-9
        limits(idx) = upper(idx) / unitControl(idx);
    elseif unitControl(idx) < -1.0e-9
        limits(idx) = lower(idx) / unitControl(idx);
    end
end

capacity = min(limits);
end
