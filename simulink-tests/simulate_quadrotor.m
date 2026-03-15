%% Quadrotor Simulation Script
% Simulates quadrotor dynamics using ODE solver
% Does not require Simulink

%% Clear workspace
clear; clc; close all;

%% Load parameters
run('quadrotor_params.m');

%% Simulation Settings
t_start = 0;        % Start time (s)
t_end = 10;         % End time (s)
dt = 0.01;          % Time step for plotting (s)

%% Initial Conditions
% State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
x0 = zeros(12, 1);
x0(3) = 0;          % Initial altitude (m) - start at ground level

%% Define Motor Speed Inputs
% Simple test: hover then ascend
w_hover = quad.w_hover;

% Define input function
motor_input = @(t) get_motor_input(t, w_hover);

%% Run Simulation using ODE45
fprintf('Running simulation...\n');
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);

[t, X] = ode45(@(t, x) quadrotor_dynamics(t, x, motor_input(t), quad), ...
               [t_start, t_end], x0, options);

fprintf('Simulation complete! Duration: %.2f s\n', t(end));

%% Extract States
x_pos = X(:, 1);
y_pos = X(:, 2);
z_pos = X(:, 3);
phi = X(:, 4);
theta = X(:, 5);
psi = X(:, 6);
vx = X(:, 7);
vy = X(:, 8);
vz = X(:, 9);
p = X(:, 10);
q = X(:, 11);
r = X(:, 12);

%% Plot Results
figure('Name', 'Quadrotor Simulation Results', 'Position', [100 100 1200 800]);

% Position subplot
subplot(2, 3, 1);
plot(t, x_pos, 'b-', t, y_pos, 'g-', t, z_pos, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs Time');
legend('x', 'y', 'z', 'Location', 'best');
grid on;

% Attitude subplot
subplot(2, 3, 2);
plot(t, rad2deg(phi), 'b-', t, rad2deg(theta), 'g-', t, rad2deg(psi), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Attitude vs Time');
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)', 'Location', 'best');
grid on;

% Velocity subplot
subplot(2, 3, 3);
plot(t, vx, 'b-', t, vy, 'g-', t, vz, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity vs Time');
legend('v_x', 'v_y', 'v_z', 'Location', 'best');
grid on;

% Angular velocity subplot
subplot(2, 3, 4);
plot(t, rad2deg(p), 'b-', t, rad2deg(q), 'g-', t, rad2deg(r), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
title('Angular Velocity vs Time');
legend('p', 'q', 'r', 'Location', 'best');
grid on;

% 3D trajectory subplot
subplot(2, 3, 5);
plot3(x_pos, y_pos, z_pos, 'b-', 'LineWidth', 2);
hold on;
plot3(x_pos(1), y_pos(1), z_pos(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(x_pos(end), y_pos(end), z_pos(end), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Trajectory');
legend('Path', 'Start', 'End', 'Location', 'best');
grid on;
axis equal;
view(45, 30);

% Altitude vs time (detailed)
subplot(2, 3, 6);
plot(t, z_pos, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs Time');
grid on;

sgtitle('Quadrotor Dynamics Simulation', 'FontSize', 14, 'FontWeight', 'bold');

%% Motor Input Function
function u = get_motor_input(t, w_hover)
    % Define motor inputs based on time
    % Inputs: [w1, w2, w3, w4] motor speeds (rad/s)

    if t < 1
        % Ramp up to hover
        u = w_hover * (t / 1) * ones(4, 1);
    elseif t < 3
        % Hover
        u = w_hover * ones(4, 1);
    elseif t < 5
        % Ascend (increase all motor speeds)
        u = w_hover * 1.1 * ones(4, 1);
    elseif t < 7
        % Roll right (increase left motors, decrease right)
        u = w_hover * [1; 1.05; 1; 0.95];
    elseif t < 9
        % Level out
        u = w_hover * ones(4, 1);
    else
        % Descend slowly
        u = w_hover * 0.95 * ones(4, 1);
    end
end

%% Save results
save('simulation_results.mat', 't', 'X', 'quad');
fprintf('Results saved to simulation_results.mat\n');
