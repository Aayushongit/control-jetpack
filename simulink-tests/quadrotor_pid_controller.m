%% Quadrotor PID Controller
% Implements cascaded PID control for quadrotor position and attitude
%
% Control Architecture:
%   Position Loop (outer) -> Attitude Loop (inner) -> Motor Mixing

classdef quadrotor_pid_controller < handle
    properties
        % Position controller gains
        Kp_pos = [2; 2; 4];      % Proportional [x, y, z]
        Ki_pos = [0.1; 0.1; 0.2]; % Integral [x, y, z]
        Kd_pos = [1.5; 1.5; 2];  % Derivative [x, y, z]

        % Attitude controller gains
        Kp_att = [8; 8; 4];      % Proportional [phi, theta, psi]
        Ki_att = [0.5; 0.5; 0.2]; % Integral [phi, theta, psi]
        Kd_att = [3; 3; 1.5];    % Derivative [phi, theta, psi]

        % Quadrotor parameters
        quad

        % Controller state (integral terms)
        int_pos = [0; 0; 0];
        int_att = [0; 0; 0];

        % Previous errors (for derivative)
        prev_err_pos = [0; 0; 0];
        prev_err_att = [0; 0; 0];
        prev_t = 0;

        % Saturation limits
        max_angle = deg2rad(30);      % Max roll/pitch command (rad)
        max_thrust = 30;               % Max total thrust (N)
        max_angular_rate = deg2rad(200); % Max angular rate (rad/s)
    end

    methods
        function obj = quadrotor_pid_controller(quad_params)
            % Constructor
            obj.quad = quad_params;
            obj.reset();
        end

        function reset(obj)
            % Reset controller states
            obj.int_pos = [0; 0; 0];
            obj.int_att = [0; 0; 0];
            obj.prev_err_pos = [0; 0; 0];
            obj.prev_err_att = [0; 0; 0];
            obj.prev_t = 0;
        end

        function u = compute_control(obj, t, state, setpoint)
            % Compute motor commands
            %
            % Inputs:
            %   t - current time
            %   state - current state [x,y,z,phi,theta,psi,vx,vy,vz,p,q,r]
            %   setpoint - desired state [x_d, y_d, z_d, psi_d]
            %
            % Output:
            %   u - motor speeds [w1, w2, w3, w4]

            % Extract state
            pos = state(1:3);
            att = state(4:6);
            vel = state(7:9);
            omega = state(10:12);

            % Extract setpoints
            pos_d = setpoint(1:3);
            psi_d = setpoint(4);

            % Calculate dt
            if obj.prev_t == 0
                dt = 0.01;
            else
                dt = t - obj.prev_t;
            end
            dt = max(dt, 1e-6);  % Prevent division by zero

            %% Position Control (Outer Loop)
            % Computes desired roll, pitch, and thrust

            % Position error
            err_pos = pos_d - pos;

            % Update integral (with anti-windup)
            obj.int_pos = obj.int_pos + err_pos * dt;
            obj.int_pos = max(min(obj.int_pos, 5), -5);  % Clamp integral

            % Derivative
            derr_pos = (err_pos - obj.prev_err_pos) / dt;

            % PID output (desired acceleration in inertial frame)
            acc_d = obj.Kp_pos .* err_pos + ...
                    obj.Ki_pos .* obj.int_pos + ...
                    obj.Kd_pos .* derr_pos;

            % Convert to thrust and attitude commands
            % Add gravity compensation for z
            acc_d(3) = acc_d(3) + obj.quad.g;

            % Desired thrust magnitude
            F_d = obj.quad.m * norm(acc_d);
            F_d = max(min(F_d, obj.max_thrust), 0);

            % Desired roll and pitch from acceleration commands
            psi = att(3);  % Current yaw

            if F_d > 0.1
                % Compute desired roll and pitch angles
                phi_d = asin((acc_d(1)*sin(psi) - acc_d(2)*cos(psi)) * obj.quad.m / F_d);
                theta_d = atan2(acc_d(1)*cos(psi) + acc_d(2)*sin(psi), acc_d(3));
            else
                phi_d = 0;
                theta_d = 0;
            end

            % Saturate angle commands
            phi_d = max(min(phi_d, obj.max_angle), -obj.max_angle);
            theta_d = max(min(theta_d, obj.max_angle), -obj.max_angle);

            % Store for next iteration
            obj.prev_err_pos = err_pos;

            %% Attitude Control (Inner Loop)
            % Computes desired moments

            att_d = [phi_d; theta_d; psi_d];

            % Attitude error
            err_att = att_d - att;

            % Wrap yaw error to [-pi, pi]
            err_att(3) = wrapToPi(err_att(3));

            % Update integral (with anti-windup)
            obj.int_att = obj.int_att + err_att * dt;
            obj.int_att = max(min(obj.int_att, 1), -1);

            % Derivative (using angular rates)
            omega_d = [0; 0; 0];  % Desired angular rates
            derr_att = omega_d - omega;

            % PID output (desired moments)
            tau = obj.Kp_att .* err_att + ...
                  obj.Ki_att .* obj.int_att + ...
                  obj.Kd_att .* derr_att;

            % Store for next iteration
            obj.prev_err_att = err_att;
            obj.prev_t = t;

            %% Motor Mixing
            % Convert thrust and moments to motor speeds
            u = obj.mix_motors(F_d, tau);
        end

        function w = mix_motors(obj, F, tau)
            % Convert force and moments to motor speeds
            %
            % Inputs:
            %   F - total thrust
            %   tau - [tau_phi; tau_theta; tau_psi] moments
            %
            % Output:
            %   w - [w1; w2; w3; w4] motor speeds (rad/s)

            k = obj.quad.k;
            b = obj.quad.b;
            L = obj.quad.L;

            % Mixing matrix (inverse allocation)
            % [F; tau_phi; tau_theta; tau_psi] = M * [w1^2; w2^2; w3^2; w4^2]

            % Motor configuration (X-config):
            %     M3 (front)
            %   M2   M4
            %     M1 (back)

            % Solve for w^2
            w1_sq = F/(4*k) - tau(2)/(2*L*k) + tau(3)/(4*b);
            w2_sq = F/(4*k) - tau(1)/(2*L*k) - tau(3)/(4*b);
            w3_sq = F/(4*k) + tau(2)/(2*L*k) + tau(3)/(4*b);
            w4_sq = F/(4*k) + tau(1)/(2*L*k) - tau(3)/(4*b);

            % Ensure non-negative
            w1_sq = max(w1_sq, 0);
            w2_sq = max(w2_sq, 0);
            w3_sq = max(w3_sq, 0);
            w4_sq = max(w4_sq, 0);

            % Take square root
            w = [sqrt(w1_sq); sqrt(w2_sq); sqrt(w3_sq); sqrt(w4_sq)];

            % Saturate to motor limits
            w = max(min(w, obj.quad.w_max), obj.quad.w_min);
        end
    end
end

%% Helper function
function angle = wrapToPi(angle)
    while angle > pi
        angle = angle - 2*pi;
    end
    while angle < -pi
        angle = angle + 2*pi;
    end
end
