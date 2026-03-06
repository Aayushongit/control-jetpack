function result = test_gain_sweep(params)
caseDefinitions = {
    struct('name', 'Soft', 'scale', 0.75)
    struct('name', 'Nominal', 'scale', 1.00)
    struct('name', 'Aggressive', 'scale', 1.25)
};

caseCount = numel(caseDefinitions);
caseResults = repmat(struct(), caseCount, 1);
severity = zeros(caseCount, 1);

for idx = 1:caseCount
    definition = caseDefinitions{idx};
    caseParams = scaled_controller(params, definition.scale);
    takeoffResult = test_takeoff(caseParams);

    altitude = takeoffResult.simulation.state(3, :);
    verticalSpeed = takeoffResult.simulation.state(6, :);
    altitudeOvershoot = max(altitude) - 1.20;
    settlingTime = jh_settling_time(takeoffResult.simulation.time, ...
        [altitude - 1.20; verticalSpeed], [0.05; 0.05]);

    caseResults(idx).name = definition.name;
    caseResults(idx).scale = definition.scale;
    caseResults(idx).simulation = takeoffResult.simulation;
    caseResults(idx).metrics.altitudeOvershoot = altitudeOvershoot;
    caseResults(idx).metrics.settlingTime = settlingTime;
    caseResults(idx).metrics.maxTiltDeg = takeoffResult.metrics.maxTiltDeg;
    caseResults(idx).metrics.finalAltitudeError = takeoffResult.metrics.finalAltitudeError;
    caseResults(idx).pass = takeoffResult.pass && altitudeOvershoot < 0.25 && ~isnan(settlingTime) && settlingTime < 9.0 && takeoffResult.metrics.maxTiltDeg < 18.0;

    settlingTimeForSeverity = settlingTime;
    if isnan(settlingTimeForSeverity)
        settlingTimeForSeverity = takeoffResult.simulation.time(end);
    end

    severity(idx) = altitudeOvershoot ...
        + 0.08 * settlingTimeForSeverity ...
        + 0.03 * takeoffResult.metrics.maxTiltDeg ...
        + takeoffResult.metrics.finalAltitudeError;
end

[~, worstIdx] = max(severity);
worstCase = caseResults(worstIdx);
worstOvershoot = max(arrayfun(@(entry) entry.metrics.altitudeOvershoot, caseResults));
settlingTimes = arrayfun(@(entry) entry.metrics.settlingTime, caseResults);
settlingTimes(isnan(settlingTimes)) = params.simulation.defaultDuration;
worstSettlingTime = max(settlingTimes);
worstMaxTiltDeg = max(arrayfun(@(entry) entry.metrics.maxTiltDeg, caseResults));

score = max(0.0, 1.0 ...
    - 1.60 * worstOvershoot ...
    - 0.06 * worstSettlingTime ...
    - 0.02 * worstMaxTiltDeg);

result.name = 'Gain Sweep';
result.simulation = caseResults(2).simulation;
result.cases = caseResults;
result.metrics.worstOvershoot = worstOvershoot;
result.metrics.worstSettlingTime = worstSettlingTime;
result.metrics.worstMaxTiltDeg = worstMaxTiltDeg;
result.metrics.worstCaseName = worstCase.name;
result.pass = all(arrayfun(@(entry) entry.pass, caseResults));
result.score = score;
result.summary = sprintf('worst %s, overshoot %.3f m, settle %.2f s', worstCase.name, worstOvershoot, worstSettlingTime);
end

function caseParams = scaled_controller(params, scale)
caseParams = params;
caseParams.controller.positionKp = scale * caseParams.controller.positionKp;
caseParams.controller.positionKd = scale * caseParams.controller.positionKd;
caseParams.controller.attitudeKp = scale * caseParams.controller.attitudeKp;
caseParams.controller.attitudeKd = scale * caseParams.controller.attitudeKd;
end
