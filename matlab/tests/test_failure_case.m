function result = test_failure_case(params)
failureParams = params;
failureParams.thrusters(1).maxThrust = 0.65 * params.thrusters(1).maxThrust;
failureParams = jh_refresh_params(failureParams);

hoverResult = test_hover(failureParams);
capability = jh_analyze_capability(failureParams);
usage = jh_command_usage(hoverResult.simulation.command, failureParams);
peakHoverTrimFraction = max(failureParams.hover.thrust ./ capability.maxThrust);

score = max(0.0, 1.0 ...
    - 1.30 * hoverResult.metrics.finalPositionError ...
    - 0.02 * hoverResult.metrics.maxTiltDeg ...
    - 1.20 * max(peakHoverTrimFraction - 0.80, 0.0));

result.name = 'Single-Thruster Degradation';
result.simulation = hoverResult.simulation;
result.capability = capability;
result.metrics.finalPositionError = hoverResult.metrics.finalPositionError;
result.metrics.maxTiltDeg = hoverResult.metrics.maxTiltDeg;
result.metrics.peakHoverTrimFraction = peakHoverTrimFraction;
result.metrics.maxCommandThrustFraction = usage.maxThrustFraction;
result.pass = hoverResult.metrics.finalPositionError < 0.18 && hoverResult.metrics.maxTiltDeg < 16.0 && peakHoverTrimFraction < 1.0;
result.score = score;
result.summary = sprintf('rear-left 65%% thrust, trim frac %.3f, pos err %.3f m', peakHoverTrimFraction, hoverResult.metrics.finalPositionError);
end
