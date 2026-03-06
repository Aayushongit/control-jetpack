function jh_plot_results(results)
testNames = default_test_order(results);

for idx = 1:numel(testNames)
    if ~isfield(results, testNames{idx})
        continue;
    end

    entry = results.(testNames{idx});
    if ~isfield(entry, 'simulation') || isempty(entry.simulation)
        continue;
    end

    time = entry.simulation.time;
    state = entry.simulation.state;
    reference = entry.simulation.reference;
    command = entry.simulation.command;

    figure('Name', entry.name);

    subplot(3, 1, 1);
    plot(time, state(1, :), 'b', time, reference.position(1, :), 'b--', ...
         time, state(2, :), 'r', time, reference.position(2, :), 'r--', ...
         time, state(3, :), 'k', time, reference.position(3, :), 'k--');
    grid on;
    xlabel('Time [s]');
    ylabel('Position [m]');
    title([entry.name ' Position Tracking']);
    legend('x', 'x ref', 'y', 'y ref', 'z', 'z ref');

    subplot(3, 1, 2);
    plot(time, state(7, :) * 180.0 / pi, 'b', ...
         time, state(8, :) * 180.0 / pi, 'r', ...
         time, state(9, :) * 180.0 / pi, 'k');
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Body Attitude');
    legend('roll', 'pitch', 'yaw');

    subplot(3, 1, 3);
    plot(time, command.thrust(1, :), 'b', ...
         time, command.thrust(2, :), 'r', ...
         time, command.thrust(3, :), 'g', ...
         time, command.thrust(4, :), 'k');
    grid on;
    xlabel('Time [s]');
    ylabel('Thrust [N]');
    title('Thruster Commands');
    legend('rear left', 'rear right', 'front left', 'front right');
end
end

function testNames = default_test_order(results)
if isfield(results, 'metadata') && isfield(results.metadata, 'testOrder')
    testNames = results.metadata.testOrder;
else
    testNames = {'hover', 'takeoff', 'landing', 'maneuverability', 'yaw'};
end
end
