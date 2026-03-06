function result = test_payload_sensitivity(params)
caseDefinitions = {
    struct('name', 'Mass +12%', 'massScale', 1.12, 'centerOfMass', [0.00; 0.00; 0.00])
    struct('name', 'Mass +8%, CG +2.5 cm x', 'massScale', 1.08, 'centerOfMass', [0.025; 0.00; 0.00])
    struct('name', 'Mass +5%, CG +2 cm y', 'massScale', 1.05, 'centerOfMass', [0.00; 0.02; 0.00])
};

caseCount = numel(caseDefinitions);
caseResults = repmat(struct(), caseCount, 1);
severity = zeros(caseCount, 1);

for idx = 1:caseCount
    definition = caseDefinitions{idx};
    caseParams = params;
    caseParams.mass = params.mass * definition.massScale;
    caseParams.centerOfMass = definition.centerOfMass;
    caseParams = jh_refresh_params(caseParams);

    hoverResult = test_hover(caseParams);
    capability = jh_analyze_capability(caseParams);
    peakHoverTrimFraction = max(caseParams.hover.thrust ./ capability.maxThrust);

    caseResults(idx).name = definition.name;
    caseResults(idx).params = caseParams;
    caseResults(idx).capability = capability;
    caseResults(idx).simulation = hoverResult.simulation;
    caseResults(idx).metrics.finalPositionError = hoverResult.metrics.finalPositionError;
    caseResults(idx).metrics.maxTiltDeg = hoverResult.metrics.maxTiltDeg;
    caseResults(idx).metrics.peakHoverTrimFraction = peakHoverTrimFraction;
    caseResults(idx).pass = hoverResult.pass && peakHoverTrimFraction < 0.95;

    severity(idx) = hoverResult.metrics.finalPositionError ...
        + 0.03 * hoverResult.metrics.maxTiltDeg ...
        + 0.80 * max(peakHoverTrimFraction - 0.70, 0.0);
end

[~, worstIdx] = max(severity);
worstCase = caseResults(worstIdx);
worstFinalPositionError = max(arrayfun(@(entry) entry.metrics.finalPositionError, caseResults));
worstMaxTiltDeg = max(arrayfun(@(entry) entry.metrics.maxTiltDeg, caseResults));
worstHoverTrimFraction = max(arrayfun(@(entry) entry.metrics.peakHoverTrimFraction, caseResults));

score = max(0.0, 1.0 ...
    - 1.40 * worstFinalPositionError ...
    - 0.02 * worstMaxTiltDeg ...
    - 1.10 * max(worstHoverTrimFraction - 0.70, 0.0));

result.name = 'Payload Sensitivity';
result.simulation = worstCase.simulation;
result.cases = caseResults;
result.metrics.worstFinalPositionError = worstFinalPositionError;
result.metrics.worstMaxTiltDeg = worstMaxTiltDeg;
result.metrics.worstHoverTrimFraction = worstHoverTrimFraction;
result.metrics.worstCaseName = worstCase.name;
result.pass = all(arrayfun(@(entry) entry.pass, caseResults));
result.score = score;
result.summary = sprintf('worst %s, pos err %.3f m, trim frac %.3f', worstCase.name, worstFinalPositionError, worstHoverTrimFraction);
end
