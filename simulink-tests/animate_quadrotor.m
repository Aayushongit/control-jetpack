function animate_quadrotor(t, X, quad, waypoints)
%ANIMATE_QUADROTOR 3D animation of quadrotor flight
%
%   animate_quadrotor(t, X, quad, waypoints)
%
%   Inputs:
%       t - time vector
%       X - state matrix (Nx12)
%       quad - parameter structure
%       waypoints - optional waypoint array (Mx4)

%% Setup figure
fig = figure('Name', 'Quadrotor Animation', 'Position', [100 100 1000 800]);
ax = axes('Parent', fig);

%% Quadrotor geometry
L = quad.L;  % Arm length

% Body vertices (in body frame)
arm_pts = [
    L,  0, 0;  % Front-right
   -L,  0, 0;  % Back-left
    0,  L, 0;  % Right
    0, -L, 0;  % Left
];

% Motor positions (X-config)
motor_pos = [
   -L,  0, 0;  % Motor 1 (back)
    0, -L, 0;  % Motor 2 (left)
    L,  0, 0;  % Motor 3 (front)
    0,  L, 0;  % Motor 4 (right)
];

% Rotor radius
r_rotor = 0.1;
theta_circle = linspace(0, 2*pi, 20);

%% Calculate animation bounds
x_range = [min(X(:,1))-1, max(X(:,1))+1];
y_range = [min(X(:,2))-1, max(X(:,2))+1];
z_range = [min(X(:,3))-0.5, max(X(:,3))+1];

% Make axes equal
max_range = max([diff(x_range), diff(y_range), diff(z_range)]);
x_center = mean(x_range);
y_center = mean(y_range);
z_center = mean(z_range);

%% Initialize plots
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');

% Ground plane
[Xg, Yg] = meshgrid(linspace(x_center-max_range/2, x_center+max_range/2, 10), ...
                     linspace(y_center-max_range/2, y_center+max_range/2, 10));
Zg = zeros(size(Xg));
surf(ax, Xg, Yg, Zg, 'FaceAlpha', 0.3, 'FaceColor', [0.5 0.8 0.5], 'EdgeColor', 'none');

% Waypoints
if nargin >= 4 && ~isempty(waypoints)
    plot3(ax, waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
          'r*', 'MarkerSize', 15, 'LineWidth', 2);
end

% Trajectory (trail)
h_trail = plot3(ax, X(1,1), X(1,2), X(1,3), 'b-', 'LineWidth', 1.5);

% Quadrotor body (arms)
h_arm1 = plot3(ax, [0,0], [0,0], [0,0], 'k-', 'LineWidth', 3);
h_arm2 = plot3(ax, [0,0], [0,0], [0,0], 'k-', 'LineWidth', 3);

% Rotors
h_rotor = gobjects(4,1);
rotor_colors = {'r', 'g', 'r', 'g'};  % CCW=red, CW=green
for i = 1:4
    h_rotor(i) = fill3(ax, zeros(1,20), zeros(1,20), zeros(1,20), ...
                       rotor_colors{i}, 'FaceAlpha', 0.5);
end

% Center body
h_body = plot3(ax, 0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Thrust vector
h_thrust = quiver3(ax, 0, 0, 0, 0, 0, 0.5, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Shadow on ground
h_shadow = plot3(ax, 0, 0, 0, 'k.', 'MarkerSize', 20);

%% Set axes
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
title(ax, 'Quadrotor Animation');
xlim(ax, [x_center-max_range/2, x_center+max_range/2]);
ylim(ax, [y_center-max_range/2, y_center+max_range/2]);
zlim(ax, [0, max(z_range(2), 1)]);
view(ax, 45, 30);

% Add time display
h_time = text(ax, x_center-max_range/2+0.5, y_center+max_range/2-0.5, max(z_range(2)), ...
              't = 0.00 s', 'FontSize', 14, 'FontWeight', 'bold');

%% Animation loop
dt_anim = 0.03;  % Animation time step
t_interp = 0:dt_anim:t(end);
X_interp = interp1(t, X, t_interp);

fprintf('Starting animation... Press Ctrl+C to stop.\n');

for i = 1:length(t_interp)
    % Current state
    pos = X_interp(i, 1:3);
    phi = X_interp(i, 4);
    theta = X_interp(i, 5);
    psi = X_interp(i, 6);

    % Rotation matrix (ZYX Euler)
    R = rotation_matrix(phi, theta, psi);

    % Transform body points to inertial frame
    arm1_body = [L 0 0; -L 0 0]';
    arm2_body = [0 L 0; 0 -L 0]';

    arm1_inertial = R * arm1_body + pos';
    arm2_inertial = R * arm2_body + pos';

    % Update arms
    set(h_arm1, 'XData', arm1_inertial(1,:), ...
                'YData', arm1_inertial(2,:), ...
                'ZData', arm1_inertial(3,:));
    set(h_arm2, 'XData', arm2_inertial(1,:), ...
                'YData', arm2_inertial(2,:), ...
                'ZData', arm2_inertial(3,:));

    % Update rotors
    for j = 1:4
        rotor_body = [r_rotor*cos(theta_circle); r_rotor*sin(theta_circle); zeros(1,20)];
        rotor_body = rotor_body + motor_pos(j,:)';
        rotor_inertial = R * rotor_body + pos';
        set(h_rotor(j), 'XData', rotor_inertial(1,:), ...
                        'YData', rotor_inertial(2,:), ...
                        'ZData', rotor_inertial(3,:));
    end

    % Update body center
    set(h_body, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

    % Update thrust vector
    thrust_dir = R * [0; 0; 0.3];
    set(h_thrust, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                  'UData', thrust_dir(1), 'VData', thrust_dir(2), 'WData', thrust_dir(3));

    % Update trail
    set(h_trail, 'XData', X_interp(1:i,1), ...
                 'YData', X_interp(1:i,2), ...
                 'ZData', X_interp(1:i,3));

    % Update shadow
    set(h_shadow, 'XData', pos(1), 'YData', pos(2), 'ZData', 0);

    % Update time display
    set(h_time, 'String', sprintf('t = %.2f s', t_interp(i)));

    % Draw
    drawnow limitrate;

    % Check if figure still exists
    if ~isvalid(fig)
        break;
    end
end

fprintf('Animation complete!\n');

end

%% Helper function
function R = rotation_matrix(phi, theta, psi)
    % ZYX Euler rotation matrix
    cphi = cos(phi); sphi = sin(phi);
    ctheta = cos(theta); stheta = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    R = [cpsi*ctheta,  cpsi*stheta*sphi - spsi*cphi,  cpsi*stheta*cphi + spsi*sphi;
         spsi*ctheta,  spsi*stheta*sphi + cpsi*cphi,  spsi*stheta*cphi - cpsi*sphi;
         -stheta,      ctheta*sphi,                   ctheta*cphi];
end
