%% Quadrotor Physical Parameters
% This file defines all physical parameters for the quadrotor model
% Source these parameters before running simulations

%% Mass and Geometry
quad.m = 1.5;           % Total mass (kg)
quad.L = 0.25;          % Arm length from center to motor (m)
quad.h = 0.05;          % Height of quadrotor body (m)

%% Moments of Inertia (kg*m^2)
quad.Ixx = 0.0142;      % Moment of inertia about x-axis (roll)
quad.Iyy = 0.0142;      % Moment of inertia about y-axis (pitch)
quad.Izz = 0.0284;      % Moment of inertia about z-axis (yaw)
quad.I = diag([quad.Ixx, quad.Iyy, quad.Izz]);  % Inertia matrix

%% Motor and Propeller Parameters
quad.k = 2.98e-6;       % Thrust coefficient (N/(rad/s)^2)
quad.b = 1.14e-7;       % Drag/torque coefficient (Nm/(rad/s)^2)
quad.Jr = 6e-5;         % Rotor inertia (kg*m^2)
quad.w_max = 1200;      % Maximum motor speed (rad/s)
quad.w_min = 0;         % Minimum motor speed (rad/s)

%% Aerodynamic Parameters
quad.Cd = 0.25;         % Drag coefficient
quad.rho = 1.225;       % Air density (kg/m^3)
quad.A = 0.1;           % Effective cross-sectional area (m^2)

%% Environment
quad.g = 9.81;          % Gravitational acceleration (m/s^2)

%% Calculated Values
% Hover motor speed
quad.w_hover = sqrt(quad.m * quad.g / (4 * quad.k));

% Mixing matrix (motor speeds to thrust/moments)
% [F; tau_phi; tau_theta; tau_psi] = M * [w1^2; w2^2; w3^2; w4^2]
quad.MixingMatrix = [
    quad.k,          quad.k,          quad.k,          quad.k;           % Thrust
    0,              -quad.L*quad.k,   0,               quad.L*quad.k;    % Roll
   -quad.L*quad.k,   0,               quad.L*quad.k,   0;                % Pitch
    quad.b,         -quad.b,          quad.b,         -quad.b            % Yaw
];

% Inverse mixing matrix (for control allocation)
quad.MixingMatrixInv = pinv(quad.MixingMatrix);

%% Display parameters
fprintf('=== Quadrotor Parameters Loaded ===\n');
fprintf('Mass: %.2f kg\n', quad.m);
fprintf('Arm Length: %.2f m\n', quad.L);
fprintf('Hover Speed: %.1f rad/s (%.0f RPM)\n', quad.w_hover, quad.w_hover*60/(2*pi));
fprintf('Max Thrust: %.2f N\n', 4*quad.k*quad.w_max^2);
fprintf('===================================\n');
