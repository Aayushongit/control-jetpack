function directions = jh_thruster_directions(pitchAngles, rollAngles)
count = numel(pitchAngles);
directions = zeros(3, count);

for idx = 1:count
    pitchAngle = pitchAngles(idx);
    rollAngle = rollAngles(idx);
    directions(:, idx) = [
        sin(pitchAngle) * cos(rollAngle);
        -sin(rollAngle);
        cos(pitchAngle) * cos(rollAngle)
    ];
end
end
