%% stewart_setup.m
%  Stewart Platform - Geometry and Parameter Definitions
%
%  6-legged, circular base Stewart (Gough-Stewart) Platform.
%  Base plate is fixed to ground; top plate moves via 6 actuated legs.
%
%  Usage:
%    >> stewart_setup   (loads all parameters into workspace)
%    >> stewart_sim     (launches 3D animation)
%
%  Author: Stewart Platform Project - 2025

% (Values from a previous run are simply overwritten)

%% =========================================================
%  GEOMETRIC PARAMETERS
%% =========================================================

deg2rad = pi / 180;

% Load parameters from central configuration file
load_config;

% Attachment point offsets in radians (converted from degrees)
alpha_b = alpha_b_deg * deg2rad;
alpha_t = alpha_t_deg * deg2rad;

%% =========================================================
%  COMPUTE ATTACHMENT POINTS
%  Each pair k shares a base cluster near phi_b and fans to
%  two top points near the SAME phi_b (no 60-deg twist).
%  This matches the photo: pairs rise straight up, no crossing.
%% =========================================================

P_base  = zeros(6, 3);   % Base attachment points (world frame)
P_top_b = zeros(6, 3);   % Top  attachment points (body frame, z = 0)

for k = 1:3
    phi_b = (k-1) * (2*pi/3);          % 0, 120, 240 deg

    % Base pair: two points close together at phi_b
    P_base(2*k-1, :) = R_base * [cos(phi_b - alpha_b), sin(phi_b - alpha_b), 0];
    P_base(2*k,   :) = R_base * [cos(phi_b + alpha_b), sin(phi_b + alpha_b), 0];

    % Top pair: centered at same phi_b (NO 60-deg offset) -> legs go straight up
    phi_t = phi_b;
    P_top_b(2*k-1, :) = R_top * [cos(phi_t - alpha_t), sin(phi_t - alpha_t), 0];
    P_top_b(2*k,   :) = R_top * [cos(phi_t + alpha_t), sin(phi_t + alpha_t), 0];
end

%% =========================================================
%  TOP ATTACHMENT POINTS IN WORLD FRAME (home position)
%% =========================================================
P_top0 = P_top_b + [0, 0, h0];

% Nominal (home) leg lengths
L0 = zeros(6,1);
for k = 1:6
    L0(k) = norm(P_top0(k,:) - P_base(k,:));
end

fprintf('=== Stewart Platform Parameters ===\n');
fprintf('Base radius     : %.3f m\n', R_base);
fprintf('Top radius      : %.3f m\n', R_top);
fprintf('Home height     : %.3f m\n', h0);
fprintf('Nominal leg L   : %.4f m\n', L0(1));
fprintf('===================================\n\n');

%% =========================================================
%  PHYSICAL PARAMETERS
%% =========================================================

% Physical parameters loaded dynamically from config.txt

L_lower = 0.65 * L0(1);   % Lower leg body length
L_upper = 0.65 * L0(1);   % Upper leg body length

m_top   = rho * pi * R_top^2  * t_plt;
m_base  = rho * pi * R_base^2 * t_plt;
m_leg   = rho * pi * (r_out^2 - r_in^2) * L_lower;

%% =========================================================
%  MOTION PROFILE  (pre-computed, sinusoidal demo)
%% =========================================================

% Simulation parameters loaded dynamically from config.txt
t_vec = (0 : dt : T_sim)';

% motion struct (used by build_simulink.m)
motion.T_sim = T_sim;
motion.dt    = dt;

N = length(t_vec);

% Motion amplitudes and frequencies (gentle phase, 0 - 9.0 s)
heave_amp  = 0.025;   % [m]
pitch_amp  = 10;      % [deg]
roll_amp   = 8;       % [deg]
heave_f    = 0.40;    % [Hz]
pitch_f    = 0.25;    % [Hz]
roll_f     = 0.35;    % [Hz]

% Aggressive tilt event (to knock ball off around t = 10 s)
t_tilt_start = 9.0;   % when tilt begins to ramp [s]
t_tilt_ramp  = 0.6;   % ramp duration [s]
max_tilt_deg = 30;    % final pitch angle [deg] -- enough to send ball over edge

% Build reference pose: [x, y, z, roll, pitch, yaw]
pos_ref = zeros(N, 6);
for i = 1:N
    t_i = t_vec(i);
    if t_i <= t_tilt_start
        % Normal gentle motion
        pos_ref(i, 3) = h0 + heave_amp * sin(2*pi * heave_f * t_i);
        pos_ref(i, 4) = roll_amp  * deg2rad * sin(2*pi * roll_f  * t_i);
        pos_ref(i, 5) = pitch_amp * deg2rad * sin(2*pi * pitch_f * t_i);
    else
        % Ramp pitch up to max_tilt_deg -> ball slides off
        alpha = min(1.0, (t_i - t_tilt_start) / t_tilt_ramp);
        pos_ref(i, 3) = h0;
        pos_ref(i, 4) = 0;
        pos_ref(i, 5) = alpha * max_tilt_deg * deg2rad;
    end
end

% Inverse kinematics -> leg lengths for each time step
leg_lengths = zeros(N, 6);
for i = 1:N
    leg_lengths(i,:) = ik_stewart(pos_ref(i,:), P_base, P_top_b);
end

% Timeseries object for Simulink From Workspace blocks
leg_ts = timeseries(leg_lengths, t_vec);
leg_ts.Name = 'LegLengths';

fprintf('Motion profile computed.\n');
fprintf('  Duration: %.1f s | Step: %.3f s | Points: %d\n', T_sim, dt, N);
fprintf('  Tilt event at t = %.1f s  (%.0f deg pitch)\n', t_tilt_start, max_tilt_deg);
fprintf('  Leg length range: [%.4f, %.4f] m\n\n', ...
        min(leg_lengths(:)), max(leg_lengths(:)));

%% =========================================================
%  STEEL BALL DYNAMICS
%
%  Phase 1 - On plate: rolling sphere (no-slip)
%    a_x = (5/7)*g*sin(pitch),   a_y = -(5/7)*g*sin(roll)
%    When ball exits plate boundary -> transition to Phase 2
%
%  Phase 2 - Free fall: parabolic trajectory in world frame
%    pos(t) = pos_exit + vel_exit*t - [0;0; 0.5*g*t^2]
%    Stops when z <= r_ball (hits floor)
%
%  ball_world(i,:) = [X, Y, Z] in world frame, stored for every step
%% =========================================================

% Ball dynamic parameters loaded dynamically from config.txt

% Effective boundary: ball center must stay within this radius on the plate
% (A ball falls when its contact point / center of mass passes the edge)
r_limit = R_top;

% Plate-frame state
ball_x  = zeros(N, 1);
ball_y  = zeros(N, 1);
ball_vx = zeros(N, 1);
ball_vy = zeros(N, 1);

% World-frame 3D position (used by animation for both on-plate and free-fall)
ball_world      = zeros(N, 3);
ball_on_plate   = true(N, 1);   % logical flag per time step

% State at the moment the ball exits the plate
fall_start_i    = N;            % step index when ball left the plate
fall_pos        = zeros(3,1);   % world position at exit
fall_vel        = zeros(3,1);   % world velocity at exit

% Initial conditions: ball at rest at plate center
ball_x(1) = 0;   ball_y(1) = 0;
ball_vx(1) = 0;  ball_vy(1) = 0;

for i = 1 : N-1
    % Platform rotation at step i
    rx_i = pos_ref(i,4);  ry_i = pos_ref(i,5);  rz_i = pos_ref(i,6);
    tx_i = pos_ref(i,1);  ty_i = pos_ref(i,2);  tz_i = pos_ref(i,3);

    Rx_i = [1,0,0; 0,cos(rx_i),-sin(rx_i); 0,sin(rx_i),cos(rx_i)];
    Ry_i = [cos(ry_i),0,sin(ry_i); 0,1,0; -sin(ry_i),0,cos(ry_i)];
    Rz_i = [cos(rz_i),-sin(rz_i),0; sin(rz_i),cos(rz_i),0; 0,0,1];
    R_i  = Rz_i * Ry_i * Rx_i;

    if ball_on_plate(i)
        % --- Phase 1: rolling on plate ---
        ax = (5/7) * g_acc * sin(ry_i) - c_roll * ball_vx(i);
        ay = -(5/7) * g_acc * sin(rx_i) - c_roll * ball_vy(i);

        ball_vx(i+1) = ball_vx(i) + ax * dt;
        ball_vy(i+1) = ball_vy(i) + ay * dt;
        ball_x(i+1)  = ball_x(i)  + ball_vx(i+1) * dt;  % Symplectic Euler
        ball_y(i+1)  = ball_y(i)  + ball_vy(i+1) * dt;  % Symplectic Euler

        % World-frame position for this step (ball sitting on plate surface)
        b_body = [ball_x(i); ball_y(i); r_ball];
        ball_world(i,:) = (R_i * b_body + [tx_i; ty_i; tz_i])';

        % Check if ball crossed the plate boundary
        if norm([ball_x(i+1), ball_y(i+1)]) > r_limit
            % Compute world-frame exit state (use velocity at step i+1)
            b_exit  = [ball_x(i+1); ball_y(i+1); r_ball];
            fall_pos = R_i * b_exit + [tx_i; ty_i; tz_i];
            fall_vel = R_i * [ball_vx(i+1); ball_vy(i+1); 0];

            fall_start_i = i + 1;
            ball_on_plate(i+1 : end) = false;

            fprintf('  Ball left plate at t = %.2f s  (plate frame pos: [%.1f, %.1f] mm)\n', ...
                t_vec(i+1), ball_x(i+1)*1000, ball_y(i+1)*1000);
        end

    else
        % --- Phase 2: free fall in world frame ---
        t_fall = t_vec(i) - t_vec(fall_start_i);
        bw = fall_pos + fall_vel * t_fall + [0; 0; -0.5 * g_acc * t_fall^2];

        % Stop at floor level (z = r_ball)
        if bw(3) < r_ball
            bw(3) = r_ball;
            % freeze ball on floor for remaining steps
            ball_world(i:end, :) = repmat(bw', N - i + 1, 1);
            break;
        end

        ball_world(i,:) = bw';
    end
end

% Fill last step if still on plate
if ball_on_plate(N)
    rx_e=pos_ref(N,4); ry_e=pos_ref(N,5); rz_e=pos_ref(N,6);
    Rx_e=[1,0,0;0,cos(rx_e),-sin(rx_e);0,sin(rx_e),cos(rx_e)];
    Ry_e=[cos(ry_e),0,sin(ry_e);0,1,0;-sin(ry_e),0,cos(ry_e)];
    Rz_e=[cos(rz_e),-sin(rz_e),0;sin(rz_e),cos(rz_e),0;0,0,1];
    R_e = Rz_e * Ry_e * Rx_e;
    ball_world(N,:) = (R_e * [ball_x(N);ball_y(N);r_ball] + pos_ref(N,1:3)')';
end

fprintf('Ball dynamics computed.\n');
fprintf('  Ball radius : %.0f mm\n', r_ball * 1000);
if any(~ball_on_plate)
    fprintf('  Fell off plate at t = %.2f s\n\n', t_vec(fall_start_i));
else
    fprintf('  Ball stayed on plate for entire simulation.\n\n');
end



%% =========================================================
%  LOCAL FUNCTION: INVERSE KINEMATICS
%% =========================================================

function L = ik_stewart(pose, P_base, P_top_body)
% IK_STEWART  Inverse kinematics for the Stewart platform.
%
%   L = ik_stewart(pose, P_base, P_top_body)
%
%   Input:
%     pose       - [x, y, z, roll, pitch, yaw]  (angles in radians)
%     P_base     - 6x3 base attachment points (world frame)
%     P_top_body - 6x3 top  attachment points (body frame, z=0 centered)
%   Output:
%     L          - 6x1 leg lengths [m]

    tx = pose(1);  ty = pose(2);  tz = pose(3);
    rx = pose(4);  ry = pose(5);  rz = pose(6);   % roll, pitch, yaw

    % ZYX Euler rotation matrix  R = Rz * Ry * Rx
    Rx = [1,      0,       0;
          0, cos(rx), -sin(rx);
          0, sin(rx),  cos(rx)];

    Ry = [cos(ry), 0, sin(ry);
          0,       1, 0;
         -sin(ry), 0, cos(ry)];

    Rz = [cos(rz), -sin(rz), 0;
          sin(rz),  cos(rz), 0;
          0,        0,       1];

    R = Rz * Ry * Rx;

    % Transform top attachment points to world frame
    t_pos = [tx; ty; tz];
    L = zeros(6,1);
    for k = 1:6
        p_world = R * P_top_body(k,:)' + t_pos;
        leg_vec = p_world - P_base(k,:)';
        L(k) = norm(leg_vec);
    end
end
