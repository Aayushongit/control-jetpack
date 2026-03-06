function jh_plot_results(results)
testNames = default_test_order(results);
pink = [0.95, 0.35, 0.65];

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
    hold on;
    plot(time, state(1, :), 'b');
    plot(time, reference.position(1, :), 'b--');
    plot(time, state(2, :), 'r');
    plot(time, reference.position(2, :), 'r--');
    plot(time, state(3, :), 'Color', pink);
    plot(time, reference.position(3, :), '--', 'Color', pink);
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Position [m]');
    title([entry.name ' Position Tracking']);
    legend('x', 'x ref', 'y', 'y ref', 'z', 'z ref');

    subplot(3, 1, 2);
    hold on;
    plot(time, state(7, :) * 180.0 / pi, 'b');
    plot(time, state(8, :) * 180.0 / pi, 'r');
    plot(time, state(9, :) * 180.0 / pi, 'Color', pink);
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Body Attitude');
    legend('roll', 'pitch', 'yaw');

    subplot(3, 1, 3);
    hold on;
    plot(time, command.thrust(1, :), 'b');
    plot(time, command.thrust(2, :), 'r');
    plot(time, command.thrust(3, :), 'g');
    plot(time, command.thrust(4, :), 'Color', pink);
    hold off;
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
