


deg2rad = pi / 180;

load_config;

alpha_b = alpha_b_deg * deg2rad;
alpha_t = alpha_t_deg * deg2rad;


P_base  = zeros(6, 3);   % Base attachment points (world frame)
P_top_b = zeros(6, 3);   % Top  attachment points (body frame, z = 0)

for k = 1:3
    phi_b = (k-1) * (2*pi/3);          % 0, 120, 240 deg

    P_base(2*k-1, :) = R_base * [cos(phi_b - alpha_b), sin(phi_b - alpha_b), 0];
    P_base(2*k,   :) = R_base * [cos(phi_b + alpha_b), sin(phi_b + alpha_b), 0];

    phi_t = phi_b;
    P_top_b(2*k-1, :) = R_top * [cos(phi_t - alpha_t), sin(phi_t - alpha_t), 0];
    P_top_b(2*k,   :) = R_top * [cos(phi_t + alpha_t), sin(phi_t + alpha_t), 0];
end

P_top0 = P_top_b + [0, 0, h0];

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



L_lower = 0.65 * L0(1);   % Lower leg body length
L_upper = 0.65 * L0(1);   % Upper leg body length

m_top   = rho * pi * R_top^2  * t_plt;
m_base  = rho * pi * R_base^2 * t_plt;
m_leg   = rho * pi * (r_out^2 - r_in^2) * L_lower;


t_vec = (0 : dt : T_sim)';

motion.T_sim = T_sim;
motion.dt    = dt;

N = length(t_vec);

heave_amp  = 0.025;   % [m]
pitch_amp  = 10;      % [deg]
roll_amp   = 8;       % [deg]
heave_f    = 0.40;    % [Hz]
pitch_f    = 0.25;    % [Hz]
roll_f     = 0.35;    % [Hz]

t_tilt_start = 9.0;   % when tilt begins to ramp [s]
t_tilt_ramp  = 0.6;   % ramp duration [s]
max_tilt_deg = 30;    % final pitch angle [deg] -- enough to send ball over edge

pos_ref = zeros(N, 6);
for i = 1:N
    t_i = t_vec(i);
    if t_i <= t_tilt_start
        pos_ref(i, 3) = h0 + heave_amp * sin(2*pi * heave_f * t_i);
        pos_ref(i, 4) = roll_amp  * deg2rad * sin(2*pi * roll_f  * t_i);
        pos_ref(i, 5) = pitch_amp * deg2rad * sin(2*pi * pitch_f * t_i);
    else
        alpha = min(1.0, (t_i - t_tilt_start) / t_tilt_ramp);
        pos_ref(i, 3) = h0;
        pos_ref(i, 4) = 0;
        pos_ref(i, 5) = alpha * max_tilt_deg * deg2rad;
    end
end

leg_lengths = zeros(N, 6);
for i = 1:N
    leg_lengths(i,:) = ik_stewart(pos_ref(i,:), P_base, P_top_b);
end

leg_ts = timeseries(leg_lengths, t_vec);
leg_ts.Name = 'LegLengths';

fprintf('Motion profile computed.\n');
fprintf('  Duration: %.1f s | Step: %.3f s | Points: %d\n', T_sim, dt, N);
fprintf('  Tilt event at t = %.1f s  (%.0f deg pitch)\n', t_tilt_start, max_tilt_deg);
fprintf('  Leg length range: [%.4f, %.4f] m\n\n', ...
        min(leg_lengths(:)), max(leg_lengths(:)));



r_limit = R_top;

ball_x  = zeros(N, 1);
ball_y  = zeros(N, 1);
ball_vx = zeros(N, 1);
ball_vy = zeros(N, 1);

ball_world      = zeros(N, 3);
ball_on_plate   = true(N, 1);   % logical flag per time step

fall_start_i    = N;            % step index when ball left the plate
fall_pos        = zeros(3,1);   % world position at exit
fall_vel        = zeros(3,1);   % world velocity at exit

ball_x(1) = 0;   ball_y(1) = 0;
ball_vx(1) = 0;  ball_vy(1) = 0;

for i = 1 : N-1
    rx_i = pos_ref(i,4);  ry_i = pos_ref(i,5);  rz_i = pos_ref(i,6);
    tx_i = pos_ref(i,1);  ty_i = pos_ref(i,2);  tz_i = pos_ref(i,3);

    Rx_i = [1,0,0; 0,cos(rx_i),-sin(rx_i); 0,sin(rx_i),cos(rx_i)];
    Ry_i = [cos(ry_i),0,sin(ry_i); 0,1,0; -sin(ry_i),0,cos(ry_i)];
    Rz_i = [cos(rz_i),-sin(rz_i),0; sin(rz_i),cos(rz_i),0; 0,0,1];
    R_i  = Rz_i * Ry_i * Rx_i;

    if ball_on_plate(i)
        ax = (5/7) * g_acc * sin(ry_i) - c_roll * ball_vx(i);
        ay = -(5/7) * g_acc * sin(rx_i) - c_roll * ball_vy(i);

        ball_vx(i+1) = ball_vx(i) + ax * dt;
        ball_vy(i+1) = ball_vy(i) + ay * dt;
        ball_x(i+1)  = ball_x(i)  + ball_vx(i+1) * dt;  % Symplectic Euler
        ball_y(i+1)  = ball_y(i)  + ball_vy(i+1) * dt;  % Symplectic Euler

        b_body = [ball_x(i); ball_y(i); r_ball];
        ball_world(i,:) = (R_i * b_body + [tx_i; ty_i; tz_i])';

        if norm([ball_x(i+1), ball_y(i+1)]) > r_limit
            b_exit  = [ball_x(i+1); ball_y(i+1); r_ball];
            fall_pos = R_i * b_exit + [tx_i; ty_i; tz_i];
            fall_vel = R_i * [ball_vx(i+1); ball_vy(i+1); 0];

            fall_start_i = i + 1;
            ball_on_plate(i+1 : end) = false;

            fprintf('  Ball left plate at t = %.2f s  (plate frame pos: [%.1f, %.1f] mm)\n', ...
                t_vec(i+1), ball_x(i+1)*1000, ball_y(i+1)*1000);
        end

    else
        t_fall = t_vec(i) - t_vec(fall_start_i);
        bw = fall_pos + fall_vel * t_fall + [0; 0; -0.5 * g_acc * t_fall^2];

        if bw(3) < r_ball
            bw(3) = r_ball;
            ball_world(i:end, :) = repmat(bw', N - i + 1, 1);
            break;
        end

        ball_world(i,:) = bw';
    end
end

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




function L = ik_stewart(pose, P_base, P_top_body)

    tx = pose(1);  ty = pose(2);  tz = pose(3);
    rx = pose(4);  ry = pose(5);  rz = pose(6);   % roll, pitch, yaw

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

    t_pos = [tx; ty; tz];
    L = zeros(6,1);
    for k = 1:6
        p_world = R * P_top_body(k,:)' + t_pos;
        leg_vec = p_world - P_base(k,:)';
        L(k) = norm(leg_vec);
    end
end
