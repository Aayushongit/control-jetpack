function x_dot = quadrotor_dynamics(t, x, u, quad)
%QUADROTOR_DYNAMICS Nonlinear dynamics of a quadrotor UAV
%
%   x_dot = quadrotor_dynamics(t, x, u, quad)
%
%   Inputs:
%       t     - Time (s) [unused, for ODE solver compatibility]
%       x     - State vector (12x1)
%               [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]'
%       u     - Input vector (4x1) - motor speeds [w1, w2, w3, w4]' (rad/s)
%       quad  - Parameter structure
%
%   Outputs:
%       x_dot - State derivative vector (12x1)
%
%   State definitions:
%       x(1:3)   = [x, y, z]           - Position (inertial frame)
%       x(4:6)   = [phi, theta, psi]   - Euler angles (roll, pitch, yaw)
%       x(7:9)   = [vx, vy, vz]        - Velocity (inertial frame)
%       x(10:12) = [p, q, r]           - Angular velocity (body frame)
%
%   Motor configuration (X-config, looking from above):
%           Front (+x)
%              M3 (CCW)
%               |
%       M2 ----+---- M4
%      (CW)    |    (CW)
%              M1 (CCW)
%           Back (-x)

%% Extract states
phi   = x(4);     % Roll angle
theta = x(5);     % Pitch angle
psi   = x(6);     % Yaw angle
vx    = x(7);     % Velocity x
vy    = x(8);     % Velocity y
vz    = x(9);     % Velocity z
p     = x(10);    % Roll rate
q     = x(11);    % Pitch rate
r     = x(12);    % Yaw rate

%% Extract inputs (motor angular velocities)
w1 = u(1);
w2 = u(2);
w3 = u(3);
w4 = u(4);

%% Saturate motor speeds
w1 = max(quad.w_min, min(quad.w_max, w1));
w2 = max(quad.w_min, min(quad.w_max, w2));
w3 = max(quad.w_min, min(quad.w_max, w3));
w4 = max(quad.w_min, min(quad.w_max, w4));

%% Extract parameters
m   = quad.m;
g   = quad.g;
Ixx = quad.Ixx;
Iyy = quad.Iyy;
Izz = quad.Izz;
L   = quad.L;
k   = quad.k;
b   = quad.b;

%% Calculate thrust and moments from motor speeds
% Total thrust (sum of all motor thrusts, pointing up in body frame)
F = k * (w1^2 + w2^2 + w3^2 + w4^2);

% Roll moment (torque about x-axis)
tau_phi = L * k * (w4^2 - w2^2);

% Pitch moment (torque about y-axis)
tau_theta = L * k * (w3^2 - w1^2);

% Yaw moment (torque about z-axis, due to motor drag)
tau_psi = b * (w1^2 - w2^2 + w3^2 - w4^2);

%% Gyroscopic effects from rotors
Omega = w1 - w2 + w3 - w4;  % Net rotor angular momentum direction

%% Trigonometric values
cphi = cos(phi);   sphi = sin(phi);
ctheta = cos(theta); stheta = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

%% Rotation matrix (body to inertial frame) - ZYX convention
R = [cpsi*ctheta,  cpsi*stheta*sphi - spsi*cphi,  cpsi*stheta*cphi + spsi*sphi;
     spsi*ctheta,  spsi*stheta*sphi + cpsi*cphi,  spsi*stheta*cphi - cpsi*sphi;
     -stheta,      ctheta*sphi,                   ctheta*cphi];

%% Translational dynamics (inertial frame)
% Thrust vector in body frame: [0; 0; F]
% Transform to inertial and add gravity
thrust_inertial = R * [0; 0; F];

ax = thrust_inertial(1) / m;
ay = thrust_inertial(2) / m;
az = thrust_inertial(3) / m - g;

%% Rotational dynamics (body frame) - Euler's equations
% Include gyroscopic effects from spinning rotors
if isfield(quad, 'Jr')
    Jr = quad.Jr;
else
    Jr = 0;
end

p_dot = (1/Ixx) * (tau_phi   - q*r*(Izz - Iyy) - Jr*q*Omega);
q_dot = (1/Iyy) * (tau_theta - p*r*(Ixx - Izz) + Jr*p*Omega);
r_dot = (1/Izz) * (tau_psi   - p*q*(Iyy - Ixx));

%% Euler angle kinematics
% Transform body rates to Euler angle rates
% Handle singularity near theta = +/- 90 degrees
if abs(ctheta) < 1e-6
    ctheta = sign(ctheta) * 1e-6;
end

phi_dot   = p + (q*sphi + r*cphi) * tan(theta);
theta_dot = q*cphi - r*sphi;
psi_dot   = (q*sphi + r*cphi) / ctheta;

%% Assemble state derivative vector
x_dot = zeros(12, 1);

% Position derivatives (velocity)
x_dot(1) = vx;
x_dot(2) = vy;
x_dot(3) = vz;

% Euler angle derivatives
x_dot(4) = phi_dot;
x_dot(5) = theta_dot;
x_dot(6) = psi_dot;

% Velocity derivatives (acceleration)
x_dot(7) = ax;
x_dot(8) = ay;
x_dot(9) = az;

% Angular velocity derivatives (angular acceleration)
x_dot(10) = p_dot;
x_dot(11) = q_dot;
x_dot(12) = r_dot;

end
