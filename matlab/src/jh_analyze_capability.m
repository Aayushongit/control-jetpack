function capability = jh_analyze_capability(params)
maxThrust = zeros(4, 1);
pitchLimits = zeros(4, 1);
rollLimits = zeros(4, 1);

for idx = 1:4
    maxThrust(idx) = params.thrusters(idx).maxThrust;
    pitchLimits(idx) = params.thrusters(idx).pitchLimit;
    rollLimits(idx) = params.thrusters(idx).rollLimit;
end

capability.hover = params.hover;
capability.maxThrust = maxThrust;
capability.totalMaxThrust = sum(maxThrust);
capability.thrustToWeight = capability.totalMaxThrust / params.weight;
capability.hoverThrottleFraction = params.weight / capability.totalMaxThrust;
capability.maxNetVerticalAcceleration = (capability.totalMaxThrust - params.weight) / params.mass;
capability.hoverMargins = maxThrust - params.hover.thrust;
capability.rearGimbalLimitDeg = pitchLimits(1) * 180.0 / pi;
capability.frontGimbalLimitDeg = pitchLimits(3) * 180.0 / pi;

singularValues = svd(params.controlEffectiveness);
capability.controlMatrix = params.controlEffectiveness;
capability.controlRank = rank(params.controlEffectiveness);
capability.controlSingularValues = singularValues;

capability.pureForceX = jh_axis_capacity(params, [1.0; 0.0; 0.0; 0.0; 0.0; 0.0]);
capability.pureForceY = jh_axis_capacity(params, [0.0; 1.0; 0.0; 0.0; 0.0; 0.0]);
capability.pureForceZ = jh_axis_capacity(params, [0.0; 0.0; 1.0; 0.0; 0.0; 0.0]);
capability.pureRollTorque = jh_axis_capacity(params, [0.0; 0.0; 0.0; 1.0; 0.0; 0.0]);
capability.purePitchTorque = jh_axis_capacity(params, [0.0; 0.0; 0.0; 0.0; 1.0; 0.0]);
capability.pureYawTorque = jh_axis_capacity(params, [0.0; 0.0; 0.0; 0.0; 0.0; 1.0]);

hoverLateralPitch = 0.0;
hoverLateralRoll = 0.0;
for idx = 1:4
    hoverLateralPitch = hoverLateralPitch + params.hover.thrust(idx) * sin(params.thrusters(idx).pitchLimit);
    hoverLateralRoll = hoverLateralRoll + params.hover.thrust(idx) * sin(params.thrusters(idx).rollLimit);
end

capability.approxMaxLateralForcePitch = hoverLateralPitch;
capability.approxMaxLateralForceRoll = hoverLateralRoll;
capability.approxMaxLateralAcceleration = max(hoverLateralPitch, hoverLateralRoll) / params.mass;

capability.realWorldNotes = {
    'This is a first-order rigid-body feasibility model derived from the MJCF.'
    'It includes thrust limits, gimbal limits, and a simple ground clamp for landing.'
    'It does not include plume interaction, actuator thermal limits, fuel use, leg contact load paths, or structural stress.'
    'Use it to reject impossible layouts early, not to certify flight readiness.'
};
end
