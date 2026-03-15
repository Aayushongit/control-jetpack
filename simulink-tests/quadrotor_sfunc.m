function [sys,x0,str,ts] = quadrotor_sfunc(t,x,u,flag,quad)
%QUADROTOR_SFUNC S-Function for quadrotor dynamics
%
%   This S-function implements the 12-state quadrotor dynamics model
%   for use in Simulink.
%
%   States (12):
%       x(1:3)   = [x, y, z]           - Position (m)
%       x(4:6)   = [phi, theta, psi]   - Euler angles (rad)
%       x(7:9)   = [vx, vy, vz]        - Velocity (m/s)
%       x(10:12) = [p, q, r]           - Angular velocity (rad/s)
%
%   Inputs (4):
%       u(1:4) = [w1, w2, w3, w4]      - Motor speeds (rad/s)
%
%   Outputs (12):
%       y = x (full state output)
%
%   Parameters:
%       quad - structure containing physical parameters

switch flag
    case 0  % Initialization
        [sys,x0,str,ts] = mdlInitializeSizes(quad);

    case 1  % Derivatives
        sys = mdlDerivatives(t,x,u,quad);

    case 2  % Update (discrete states - unused)
        sys = [];

    case 3  % Outputs
        sys = mdlOutputs(t,x,u);

    case 4  % GetTimeOfNextVarHit (unused)
        sys = [];

    case 9  % Terminate
        sys = [];

    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

end

%% ========== Initialization ==========
function [sys,x0,str,ts] = mdlInitializeSizes(quad)
    sizes = simsizes;

    sizes.NumContStates  = 12;   % 12 continuous states
    sizes.NumDiscStates  = 0;    % No discrete states
    sizes.NumOutputs     = 12;   % 12 outputs (full state)
    sizes.NumInputs      = 4;    % 4 inputs (motor speeds)
    sizes.DirFeedthrough = 0;    % No direct feedthrough
    sizes.NumSampleTimes = 1;    % Single sample time

    sys = simsizes(sizes);

    % Initial conditions
    x0 = zeros(12, 1);

    % No state ordering string
    str = [];

    % Continuous sample time
    ts = [0 0];
end

%% ========== Derivatives ==========
function sys = mdlDerivatives(t,x,u,quad)
    % Extract states
    phi   = x(4);     % Roll
    theta = x(5);     % Pitch
    psi   = x(6);     % Yaw
    vx    = x(7);
    vy    = x(8);
    vz    = x(9);
    p     = x(10);    % Roll rate
    q     = x(11);    % Pitch rate
    r     = x(12);    % Yaw rate

    % Extract inputs
    w1 = max(0, u(1));
    w2 = max(0, u(2));
    w3 = max(0, u(3));
    w4 = max(0, u(4));

    % Parameters
    m   = quad.m;
    g   = quad.g;
    Ixx = quad.Ixx;
    Iyy = quad.Iyy;
    Izz = quad.Izz;
    L   = quad.L;
    k   = quad.k;
    b   = quad.b;

    % Forces and moments
    F = k * (w1^2 + w2^2 + w3^2 + w4^2);
    tau_phi = L * k * (w4^2 - w2^2);
    tau_theta = L * k * (w3^2 - w1^2);
    tau_psi = b * (w1^2 - w2^2 + w3^2 - w4^2);

    % Rotation matrix
    cphi = cos(phi); sphi = sin(phi);
    ctheta = cos(theta); stheta = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    R = [cpsi*ctheta,  cpsi*stheta*sphi - spsi*cphi,  cpsi*stheta*cphi + spsi*sphi;
         spsi*ctheta,  spsi*stheta*sphi + cpsi*cphi,  spsi*stheta*cphi - cpsi*sphi;
         -stheta,      ctheta*sphi,                   ctheta*cphi];

    % Translational acceleration
    thrust_body = [0; 0; F];
    thrust_inertial = R * thrust_body;
    ax = thrust_inertial(1) / m;
    ay = thrust_inertial(2) / m;
    az = thrust_inertial(3) / m - g;

    % Rotational dynamics
    p_dot = (1/Ixx) * (tau_phi   - q*r*(Izz - Iyy));
    q_dot = (1/Iyy) * (tau_theta - p*r*(Ixx - Izz));
    r_dot = (1/Izz) * (tau_psi   - p*q*(Iyy - Ixx));

    % Euler angle kinematics
    if abs(ctheta) < 1e-6
        ctheta = sign(ctheta) * 1e-6;
    end
    phi_dot   = p + (q*sphi + r*cphi) * tan(theta);
    theta_dot = q*cphi - r*sphi;
    psi_dot   = (q*sphi + r*cphi) / ctheta;

    % State derivatives
    sys = [vx; vy; vz; phi_dot; theta_dot; psi_dot; ax; ay; az; p_dot; q_dot; r_dot];
end

%% ========== Outputs ==========
function sys = mdlOutputs(t,x,u)
    % Output full state
    sys = x;
end
