function settlingTime = jh_settling_time(time, signals, thresholds)
if isempty(time)
    settlingTime = NaN;
    return;
end

if isvector(signals)
    signals = reshape(signals, 1, []);
end

if isscalar(thresholds)
    thresholds = repmat(thresholds, size(signals, 1), 1);
end

within = true(1, numel(time));
for idx = 1:size(signals, 1)
    within = within & abs(signals(idx, :)) <= thresholds(idx);
end

stableFrom = within;
for idx = numel(time) - 1:-1:1
    stableFrom(idx) = within(idx) && stableFrom(idx + 1);
end

settlingIndex = find(stableFrom, 1);
if isempty(settlingIndex)
    settlingTime = NaN;
else
    settlingTime = time(settlingIndex);
end
end
