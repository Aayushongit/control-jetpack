%% Quadrotor Linearization
% Linearizes quadrotor dynamics about hover equilibrium
% for LQR and state-space controller design

%% Clear workspace
clear; clc; close all;

%% Load parameters
run('quadrotor_params.m');

%% Equilibrium Point (Hover)
% State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
x_eq = zeros(12, 1);
x_eq(3) = 1;  % Hover at 1m altitude

% Motor speeds at hover
w_eq = quad.w_hover * ones(4, 1);

%% Linearized State-Space Matrices
% x_dot = A*x + B*u (about hover equilibrium)

% Extract parameters
m = quad.m;
g = quad.g;
Ixx = quad.Ixx;
Iyy = quad.Iyy;
Izz = quad.Izz;
k = quad.k;
L = quad.L;
b = quad.b;

% Hover thrust
F_hover = m * g;
w_hover = quad.w_hover;

%% A Matrix (12x12)
% Linearized about hover: phi=0, theta=0, small angles
A = zeros(12, 12);

% Position derivatives = velocity
A(1, 7) = 1;  % x_dot = vx
A(2, 8) = 1;  % y_dot = vy
A(3, 9) = 1;  % z_dot = vz

% Euler angle derivatives = angular rates (simplified at hover)
A(4, 10) = 1;  % phi_dot = p
A(5, 11) = 1;  % theta_dot = q
A(6, 12) = 1;  % psi_dot = r

% Velocity derivatives depend on attitude
% At hover: ax = g*theta, ay = -g*phi (for small angles)
A(7, 5) = g;   % vx_dot = g*theta (pitch forward = accelerate forward)
A(8, 4) = -g;  % vy_dot = -g*phi (roll right = accelerate left... or opposite)

% Angular acceleration (no coupling at equilibrium for symmetric drone)
% p_dot, q_dot, r_dot depend only on moments

%% B Matrix (12x4)
% Input: motor speed perturbations [dw1, dw2, dw3, dw4]
B = zeros(12, 4);

% Thrust affects z acceleration
% dF = 2*k*w_hover * sum(dw)
dFdw = 2 * k * w_hover;  % Thrust sensitivity to motor speed

% z_dot_dot = dF/m (for each motor)
B(9, 1) = dFdw / m;
B(9, 2) = dFdw / m;
B(9, 3) = dFdw / m;
B(9, 4) = dFdw / m;

% Moment sensitivities
dtau_dw = 2 * k * L * w_hover;  % Roll/pitch moment sensitivity
dtau_psi_dw = 2 * b * w_hover;  % Yaw moment sensitivity

% Roll (phi) acceleration from tau_phi
% tau_phi = L*k*(w4^2 - w2^2), d(tau_phi)/dw4 = 2*L*k*w_hover
B(10, 2) = -dtau_dw / Ixx;  % Motor 2 (left)
B(10, 4) = dtau_dw / Ixx;   % Motor 4 (right)

% Pitch (theta) acceleration from tau_theta
% tau_theta = L*k*(w3^2 - w1^2)
B(11, 1) = -dtau_dw / Iyy;  % Motor 1 (back)
B(11, 3) = dtau_dw / Iyy;   % Motor 3 (front)

% Yaw (psi) acceleration from tau_psi
% tau_psi = b*(w1^2 - w2^2 + w3^2 - w4^2)
B(12, 1) = dtau_psi_dw / Izz;
B(12, 2) = -dtau_psi_dw / Izz;
B(12, 3) = dtau_psi_dw / Izz;
B(12, 4) = -dtau_psi_dw / Izz;

%% C Matrix (Output = full state)
C = eye(12);

%% D Matrix (No direct feedthrough)
D = zeros(12, 4);

%% Create State-Space Model
sys = ss(A, B, C, D);

% Name states and inputs
sys.StateName = {'x', 'y', 'z', 'phi', 'theta', 'psi', ...
                 'vx', 'vy', 'vz', 'p', 'q', 'r'};
sys.InputName = {'dw1', 'dw2', 'dw3', 'dw4'};
sys.OutputName = sys.StateName;

%% Display results
fprintf('=== Linearized Quadrotor Model ===\n\n');

fprintf('A matrix (12x12):\n');
disp(A);

fprintf('B matrix (12x4):\n');
disp(B);

%% Check controllability
Co = ctrb(A, B);
rank_Co = rank(Co);
fprintf('Controllability matrix rank: %d (should be 12 for full controllability)\n', rank_Co);

if rank_Co == 12
    fprintf('System is fully controllable!\n\n');
else
    fprintf('WARNING: System is NOT fully controllable!\n\n');
end

%% Check observability
Ob = obsv(A, C);
rank_Ob = rank(Ob);
fprintf('Observability matrix rank: %d\n', rank_Ob);

%% Eigenvalues (poles)
poles = eig(A);
fprintf('\nOpen-loop poles:\n');
disp(poles);

%% Design LQR Controller
fprintf('=== LQR Controller Design ===\n\n');

% Weight matrices
Q = diag([10, 10, 20, ...      % Position weights (x, y, z)
          5, 5, 5, ...         % Attitude weights (phi, theta, psi)
          1, 1, 1, ...         % Velocity weights
          0.1, 0.1, 0.1]);     % Angular rate weights

R = 0.1 * eye(4);              % Control effort weight

% LQR gain
[K_lqr, S, poles_cl] = lqr(A, B, Q, R);

fprintf('LQR Gain K (4x12):\n');
disp(K_lqr);

fprintf('Closed-loop poles:\n');
disp(poles_cl);

%% Save linearized model
save('linearized_model.mat', 'A', 'B', 'C', 'D', 'sys', 'K_lqr', 'Q', 'R', 'x_eq', 'w_eq', 'quad');
fprintf('Linearized model saved to linearized_model.mat\n');

%% Plot pole locations
figure('Name', 'Pole Locations');
subplot(1,2,1);
plot(real(poles), imag(poles), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
xline(0, 'k--');
xlabel('Real'); ylabel('Imaginary');
title('Open-Loop Poles');
grid on;
axis equal;

subplot(1,2,2);
plot(real(poles_cl), imag(poles_cl), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
xline(0, 'k--');
xlabel('Real'); ylabel('Imaginary');
title('Closed-Loop Poles (LQR)');
grid on;
axis equal;

sgtitle('Quadrotor Pole Placement');
