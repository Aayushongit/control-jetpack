%% Controlled Quadrotor Simulation
% Simulates quadrotor with PID control following waypoints

%% Clear workspace
clear; clc; close all;

%% Load parameters
run('quadrotor_params.m');

%% Create controller
controller = quadrotor_pid_controller(quad);

%% Simulation Settings
t_end = 30;         % Simulation duration (s)
dt = 0.01;          % Time step (s)
t = 0:dt:t_end;
N = length(t);

%% Initial Conditions
x0 = zeros(12, 1);  % Start at origin

%% Define Waypoints
waypoints = [
    0, 0, 2, 0;       % Takeoff to 2m altitude
    2, 0, 2, 0;       % Move forward
    2, 2, 2, pi/2;    % Move right, rotate 90 deg
    0, 2, 3, pi;      % Move back, ascend
    0, 0, 2, 0;       % Return to start
    0, 0, 0, 0;       % Land
];

%% Allocate arrays
X = zeros(N, 12);
X(1, :) = x0';
U = zeros(N, 4);
setpoints = zeros(N, 4);

%% Waypoint tracking
waypoint_idx = 1;
waypoint_threshold = 0.3;  % Distance to switch waypoint (m)

%% Run Simulation
fprintf('Running controlled simulation...\n');
progress_bar = waitbar(0, 'Simulating...');

for i = 2:N
    curr_t = t(i);

    % Get current state
    state = X(i-1, :)';

    % Check if waypoint reached
    pos = state(1:3);
    wp = waypoints(waypoint_idx, 1:3)';
    dist = norm(pos - wp);

    if dist < waypoint_threshold && waypoint_idx < size(waypoints, 1)
        waypoint_idx = waypoint_idx + 1;
        fprintf('Waypoint %d reached at t=%.1fs\n', waypoint_idx-1, curr_t);
    end

    % Current setpoint
    setpoint = waypoints(waypoint_idx, :)';
    setpoints(i, :) = setpoint';

    % Compute control
    u = controller.compute_control(curr_t, state, setpoint);
    U(i, :) = u';

    % Integrate dynamics (RK4)
    k1 = quadrotor_dynamics(curr_t, state, u, quad);
    k2 = quadrotor_dynamics(curr_t + dt/2, state + dt/2*k1, u, quad);
    k3 = quadrotor_dynamics(curr_t + dt/2, state + dt/2*k2, u, quad);
    k4 = quadrotor_dynamics(curr_t + dt, state + dt*k3, u, quad);

    X(i, :) = (state + dt/6*(k1 + 2*k2 + 2*k3 + k4))';

    % Update progress
    if mod(i, 100) == 0
        waitbar(i/N, progress_bar);
    end
end

close(progress_bar);
fprintf('Simulation complete!\n');

%% Plot Results
figure('Name', 'Controlled Quadrotor Simulation', 'Position', [50 50 1400 900]);

% 3D Trajectory
subplot(2, 3, 1);
plot3(X(:,1), X(:,2), X(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
plot3(X(1,1), X(1,2), X(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(X(end,1), X(end,2), X(end,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory');
legend('Path', 'Waypoints', 'Start', 'End', 'Location', 'best');
grid on; axis equal; view(45, 30);

% Position tracking
subplot(2, 3, 2);
plot(t, X(:,1), 'b-', t, X(:,2), 'g-', t, X(:,3), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, setpoints(:,1), 'b--', t, setpoints(:,2), 'g--', t, setpoints(:,3), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Position (m)');
title('Position Tracking');
legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'best');
grid on;

% Attitude
subplot(2, 3, 3);
plot(t, rad2deg(X(:,4)), 'b-', t, rad2deg(X(:,5)), 'g-', t, rad2deg(X(:,6)), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(setpoints(:,4)), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Attitude');
legend('\phi', '\theta', '\psi', '\psi_d', 'Location', 'best');
grid on;

% Motor speeds
subplot(2, 3, 4);
plot(t, U(:,1), t, U(:,2), t, U(:,3), t, U(:,4), 'LineWidth', 1);
hold on;
yline(quad.w_hover, 'k--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Motor Speed (rad/s)');
title('Motor Commands');
legend('w_1', 'w_2', 'w_3', 'w_4', 'Hover', 'Location', 'best');
grid on;

% Velocity
subplot(2, 3, 5);
plot(t, X(:,7), 'b-', t, X(:,8), 'g-', t, X(:,9), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity');
legend('v_x', 'v_y', 'v_z', 'Location', 'best');
grid on;

% Position error
subplot(2, 3, 6);
pos_error = sqrt((X(:,1)-setpoints(:,1)).^2 + (X(:,2)-setpoints(:,2)).^2 + (X(:,3)-setpoints(:,3)).^2);
plot(t, pos_error, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Error (3D)');
grid on;

sgtitle('PID Controlled Quadrotor Simulation', 'FontSize', 14, 'FontWeight', 'bold');

%% Save results
save('controlled_simulation_results.mat', 't', 'X', 'U', 'setpoints', 'waypoints', 'quad');
fprintf('Results saved to controlled_simulation_results.mat\n');

%% Animation (optional)
fprintf('\nPress any key to start animation...\n');
pause;

animate_quadrotor(t, X, quad, waypoints);
