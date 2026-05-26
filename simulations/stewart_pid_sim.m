%% stewart_pid_sim.m
%  Stewart Platform - Classic PID Ball Balancing Simulation
%
%  PID controller keeps the steel ball at plate center (0,0).
%  Control outputs: platform pitch and roll angles.
%
%  Control law (plate body frame):
%    ball accel X = (5/7)*g*sin(pitch)  ->  pitch_cmd = PID(-x_ball)
%    ball accel Y = -(5/7)*g*sin(roll)  ->  roll_cmd  = PID(+y_ball)
%
%  Usage:  >> stewart_pid_sim

speed_mult = 1.0;   % fixed 1× real-time playback

%% Load platform geometry and parameters
run('stewart_setup.m');
%% =========================================================
%  INITIAL CONDITIONS
%  Disturbance table generated dynamically for 30s
%% =========================================================
seed_str = input('Enter random seed for disturbances (integer, default 1): ', 's');
if isempty(seed_str), seed = 1; else, seed = str2double(seed_str); end

% Generate random disturbances and sensor noise for T_sim = 30s
T_sim_pid = 30;
disturb_table = generate_disturbances(seed, T_sim_pid);
noise_table = generate_sensor_noise(seed, T_sim_pid, dt);

% Temporarily override N and t_vec for 30 seconds
N = round(T_sim_pid / dt);
t_vec = linspace(0, T_sim_pid, N)';
pos_ref_pid = zeros(N, 6);
pos_ref_pid(:, 3) = h0;   % fixed height

%% =========================================================
%  PID PARAMETERS
%  Output: tilt angle [rad] per meter of ball displacement
%  Tune Kp, Ki, Kd here if response is too slow / oscillates.
%% =========================================================
Kp = 20;    % proportional gain [rad/m]
Ki = 3;    % integral gain     [rad/(m*s)]
Kd = 1;    % derivative gain   [rad*s/m]

max_tilt = 30 * deg2rad;   % physical tilt limit [rad]

%% =========================================================
%  RUN PHYSICS VIA SHARED ENGINE (simulate_ball)
%% =========================================================
fprintf('Running PID closed-loop simulation...\n');

sim_params = struct();
sim_params.dt       = dt;
sim_params.T_sim    = T_sim_pid;
sim_params.g_acc    = g_acc;
sim_params.c_roll   = c_roll;
sim_params.r_limit  = r_limit;
sim_params.max_tilt = max_tilt;
sim_params.ball_x0  = 0.05;
sim_params.ball_y0  = 0.03;

sim_result = simulate_ball(Kp, Ki, Kd, sim_params, disturb_table, noise_table);

% Unpack results into workspace variables expected by animation code
ball_x_pid  = sim_result.ball_x;
ball_y_pid  = sim_result.ball_y;
ball_vx_pid = sim_result.ball_vx;
ball_vy_pid = sim_result.ball_vy;
pitch_log   = sim_result.pitch_cmd;
roll_log    = sim_result.roll_cmd;
N           = sim_result.N;
t_vec       = sim_result.t_vec;

% Build pos_ref for animation
pos_ref_pid = zeros(N, 6);
pos_ref_pid(:, 3) = h0;
pos_ref_pid(2:end, 4) = roll_log(1:end-1);
pos_ref_pid(2:end, 5) = pitch_log(1:end-1);

% Print disturbance log
for d = 1:size(disturb_table, 1)
    idx_d = find(abs(t_vec - disturb_table(d,1)) < dt/2, 1);
    if ~isempty(idx_d)
        fprintf('  Disturbance #%d at t = %.2f s  (kick: [%.2f, %.2f] m/s)\n', ...
            d, disturb_table(d,1), disturb_table(d,2), disturb_table(d,3));
    end
end



% Build world-frame ball positions + pre-compute animation geometry
% Includes plate boundary check: ball falls off if it exits r_limit
fprintf('Pre-computing animation data...\n');
theta_c  = linspace(0, 2*pi, 20)';   % 20 pts — visually smooth, 58% fewer fill3 vertices
n_ring   = numel(theta_c);             % ring resolution — change theta_c only, rest adapts
cos_tc   = cos(theta_c); sin_tc = sin(theta_c);

ball_world_pid  = zeros(N, 3);
ball_on_plate_pid = true(N, 1);
R_all_pid       = zeros(3,3,N);
top_ring_pid    = zeros(n_ring,3,N);
P_top_pid       = zeros(6,3,N);
leg_x_pid       = nan(18,N);
leg_y_pid       = nan(18,N);
leg_z_pid       = nan(18,N);

fall_start_pid  = N;
fall_pos_pid    = zeros(3,1);
fall_vel_pid    = zeros(3,1);

for i = 1:N
    rx_i=pos_ref_pid(i,4); ry_i=pos_ref_pid(i,5); rz_i=pos_ref_pid(i,6);
    Rx_i=[1,0,0;0,cos(rx_i),-sin(rx_i);0,sin(rx_i),cos(rx_i)];
    Ry_i=[cos(ry_i),0,sin(ry_i);0,1,0;-sin(ry_i),0,cos(ry_i)];
    Rz_i=[cos(rz_i),-sin(rz_i),0;sin(rz_i),cos(rz_i),0;0,0,1];
    R_i = Rz_i*Ry_i*Rx_i;
    R_all_pid(:,:,i) = R_i;

    ring_local = R_top * [cos_tc, sin_tc, zeros(n_ring,1)]';
    ring_world = R_i * ring_local;
    top_ring_pid(:,:,i) = (ring_world + [0;0;h0])';
    for k = 1:6
        P_top_pid(k,:,i) = (R_i*P_top_b(k,:)' + [0;0;h0])';
        idx = (k-1)*3 + 1;
        leg_x_pid(idx,  i) = P_base(k,1);  leg_x_pid(idx+1,i) = P_top_pid(k,1,i);
        leg_y_pid(idx,  i) = P_base(k,2);  leg_y_pid(idx+1,i) = P_top_pid(k,2,i);
        leg_z_pid(idx,  i) = P_base(k,3);  leg_z_pid(idx+1,i) = P_top_pid(k,3,i);
    end

    % Ball world position (with fall-off check)
    if ball_on_plate_pid(i)
        bw = R_i*[ball_x_pid(i);ball_y_pid(i);r_ball] + [0;0;h0];
        ball_world_pid(i,:) = bw';

        % Check plate boundary
        if norm([ball_x_pid(i), ball_y_pid(i)]) > r_limit && i < N
            ball_on_plate_pid(i+1:end) = false;
            fall_start_pid = i + 1;
            fall_pos_pid   = bw;
            fall_vel_pid   = R_i * [ball_vx_pid(i); ball_vy_pid(i); 0];
            fprintf('  Ball left plate (PID) at t = %.2f s\n', t_vec(i));
        end
    else
        % Free fall in world frame
        t_fall = t_vec(i) - t_vec(fall_start_pid);
        bw = fall_pos_pid + fall_vel_pid*t_fall + [0;0;-0.5*g_acc*t_fall^2];

        if bw(3) < r_ball   % hit floor
            bw(3) = r_ball;
            ball_world_pid(i:end,:) = repmat(bw', N-i+1, 1);
            break;
        end
        ball_world_pid(i,:) = bw';
    end
end
fprintf('Done.\n');


fprintf('  Done. Settling time (approx): ');
settled = find(sqrt(ball_x_pid.^2 + ball_y_pid.^2) < 0.005 & t_vec > 0.5, 1);
if ~isempty(settled)
    fprintf('%.2f s\n\n', t_vec(settled));
else
    fprintf('not settled within %.1f s\n\n', T_sim);
end

%% =========================================================
%  ANIMATION
%% =========================================================
fig = figure('Name', 'Stewart Platform - PID Ball Balancing', ...
             'NumberTitle', 'off', 'Color', [0.10 0.10 0.13], ...
             'Position', [100, 50, 1000, 700]);
set(fig, 'Renderer', 'opengl', 'RendererMode', 'manual');  % force GPU path, no auto-switch

ax = axes('Parent', fig, 'Color', [0.08 0.08 0.10], ...
          'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
          'ZColor', [0.7 0.7 0.7], 'GridColor', [0.35 0.35 0.35], ...
          'GridAlpha', 0.4);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax, 40, 28);

lim = R_base * 1.5;
xlim(ax,[-lim lim]); ylim(ax,[-lim lim]); zlim(ax,[0 h0*2.4]);
axis(ax, 'manual');   % freeze limits — prevents costly recalculation on every drawnow
xlabel(ax,'X [m]','Color',[0.8 0.8 0.8]);
ylabel(ax,'Y [m]','Color',[0.8 0.8 0.8]);
zlabel(ax,'Z [m]','Color',[0.8 0.8 0.8]);
title(ax,'PID Ball Balancing - Stewart Platform', ...
      'Color',[0.95 0.95 0.95],'FontSize',13,'FontWeight','bold');

% Info text
time_txt  = text(ax,-lim*.9,-lim*.9,h0*2.2,'t = 0.00 s', ...
    'Color',[0.9 0.9 0.2],'FontSize',10,'FontWeight','bold');
mode_txt  = text(ax,-lim*.9,-lim*.9,h0*2.05,'PID: ON', ...
    'Color',[0.3 1.0 0.4],'FontSize',10,'FontWeight','bold');

global WIND_TYPE;
global WIND_C_RATIO;
global WIND_R_RATIO;
if isempty(WIND_TYPE), WIND_TYPE = 'chaotic'; end
if strcmpi(WIND_TYPE, 'combined')
    if isempty(WIND_C_RATIO), WIND_C_RATIO = 0.3; end
    if isempty(WIND_R_RATIO), WIND_R_RATIO = 1.0; end
    wind_disp = sprintf('Wind: COMBINED (c:%.1f r:%.1f)', WIND_C_RATIO, WIND_R_RATIO);
else
    wind_disp = sprintf('Wind: %s', upper(WIND_TYPE));
end
wind_txt = text(ax,-lim*.9,-lim*.9,h0*1.90, wind_disp, ...
    'Color',[0.8 0.8 0.8],'FontSize',10,'FontWeight','bold');

kp_txt = text(ax, lim*0.5, lim*0.8, h0*2.2, sprintf('Kp: %.2f', Kp), 'Color',[0.3 1.0 0.4],'FontSize',11,'FontWeight','bold');
ki_txt = text(ax, lim*0.5, lim*0.8, h0*2.0, sprintf('Ki: %.2f', Ki), 'Color',[0.3 0.8 1.0],'FontSize',11,'FontWeight','bold');
kd_txt = text(ax, lim*0.5, lim*0.8, h0*1.8, sprintf('Kd: %.2f', Kd), 'Color',[1.0 0.8 0.2],'FontSize',11,'FontWeight','bold');

leg_colors = [0.30,0.60,1.00; 1.00,0.40,0.30; 0.30,0.90,0.40;
              1.00,0.80,0.10; 0.80,0.30,1.00; 0.20,0.90,0.90];

% Base plate (static)
fill3(ax, R_base*cos_tc', R_base*sin_tc', zeros(1,n_ring), ...
      [0.35 0.35 0.45],'FaceAlpha',0.6,'EdgeColor',[0.6 0.6 0.7],'LineWidth',1.5);
plot3(ax, P_base(:,1), P_base(:,2), P_base(:,3), 'o', ...
      'Color',[0.9 0.9 0.9],'MarkerSize',7,'MarkerFaceColor',[0.6 0.6 0.7]);

% Target marker at plate center
plot3(ax, 0, 0, h0+0.001, '+', 'Color', [0.3 1.0 0.3], ...
      'MarkerSize', 16, 'LineWidth', 2.5);

% Top plate (dynamic)
tr0 = top_ring_pid(:,:,1);
h_top = fill3(ax,tr0(:,1)',tr0(:,2)',tr0(:,3)', ...
              [0.20,0.45,0.80],'FaceAlpha',0.75,'EdgeColor',[0.5 0.7 1.0],'LineWidth',2);
h_top_pts = plot3(ax,P_top_pid(:,1,1),P_top_pid(:,2,1),P_top_pid(:,3,1), ...
                  'o','Color',[0.9 0.9 0.9],'MarkerSize',7,'MarkerFaceColor',[0.3 0.6 0.9]);

% All 6 legs
h_legs = gobjects(6,1);
for k = 1:6
    idx = (k-1)*3 + 1;
    h_legs(k) = plot3(ax, leg_x_pid(idx:idx+1,1), leg_y_pid(idx:idx+1,1), leg_z_pid(idx:idx+1,1), ...
        '-', 'Color', leg_colors(k,:), 'LineWidth', 2.8);
end

% Platform center dot (simple marker, no quiver)
h_ctr = plot3(ax,0,0,h0,'o','Color',[1 0.8 0.2],'MarkerSize',8, ...
    'MarkerFaceColor',[1 0.8 0.2],'LineWidth',1);

% Steel ball — flat shading (no lighting = much faster), sphere(6) = 49 vs 81 vertices
[sp_x,sp_y,sp_z] = sphere(6);
ball_surf = surf(ax, sp_x*r_ball, sp_y*r_ball, sp_z*r_ball+h0+r_ball, ...
    'FaceColor',[0.78 0.78 0.85],'EdgeColor','none','FaceLighting','none');

% Trail
trail_len = 15;
trail_buf = repmat(ball_world_pid(1,:), trail_len, 1);
h_trail   = plot3(ax,trail_buf(:,1),trail_buf(:,2),trail_buf(:,3), ...
    '-','Color',[0.3 0.9 1.0 0.7],'LineWidth',1.5);

fprintf('Animation starting...  Close figure to stop.\n\n');
% Smooth real-time animation logic:
% No limitrate, raw drawnow. Render every 2 frames (50 fps) or 3 frames (33 fps)
RENDER_EVERY = 3; 
tic;
for i = 1:N
    if ~ishandle(fig), break; end
    if mod(i, RENDER_EVERY) ~= 0 && i ~= N, continue; end

    tr = top_ring_pid(:,:,i);
    set(h_top,     'XData',tr(:,1)',         'YData',tr(:,2)',         'ZData',tr(:,3)');
    set(h_top_pts, 'XData',P_top_pid(:,1,i),'YData',P_top_pid(:,2,i),'ZData',P_top_pid(:,3,i));
    for k = 1:6
        idx = (k-1)*3 + 1;
        set(h_legs(k), 'XData',leg_x_pid(idx:idx+1,i), 'YData',leg_y_pid(idx:idx+1,i), 'ZData',leg_z_pid(idx:idx+1,i));
    end

    tz = pos_ref_pid(i,3);
    set(h_ctr,'XData',0,'YData',0,'ZData',tz);

    bw = ball_world_pid(i,:);
    set(ball_surf,'XData',bw(1)+sp_x*r_ball,'YData',bw(2)+sp_y*r_ball,'ZData',bw(3)+sp_z*r_ball);
    trail_buf(1:end-1,:) = trail_buf(2:end,:);
    trail_buf(end,:) = bw;
    tc = [0.3, 0.9, 1.0, 0.7];                      % cyan = on plate
    if ~ball_on_plate_pid(i), tc = [1.0,0.15,0.15,0.9]; end  % red = free fall
    set(h_trail,'XData',trail_buf(:,1),'YData',trail_buf(:,2),'ZData',trail_buf(:,3),'Color',tc);

    set(time_txt,'String',sprintf('t = %.2f s', t_vec(i)));
    is_disturb = any(abs(t_vec(i) - disturb_table(:,1)) < 0.4);
    if is_disturb
        set(mode_txt,'String','DISTURBANCE!','Color',[1.0 0.3 0.2]);
    else
        set(mode_txt,'String','PID: ON','Color',[0.3 1.0 0.4]);
    end

    drawnow; % raw drawnow for max smoothness

    % Real-time synchronization
    elapsed = toc;
    sim_time = t_vec(i);
    if elapsed < sim_time
        slack = sim_time - elapsed;
        if slack > 0.020
            pause(slack - 0.015);
        end
    end
end
fprintf('Animation complete.\n');

%% =========================================================
%  RESULTS PLOTS
%% =========================================================
figure('Name','PID Results','Color',[0.10 0.10 0.13],'NumberTitle','off', ...
       'Position',[100 50 900 650]);

% Ball position
subplot(3,1,1);
set(gca,'Color',[0.08 0.08 0.10],'XColor',[0.8 0.8 0.8],'YColor',[0.8 0.8 0.8],'GridColor',[0.4 0.4 0.4]);
hold on; grid on;
plot(t_vec, ball_x_pid*100, 'Color',[0.3 0.6 1.0],'LineWidth',1.8,'DisplayName','X');
plot(t_vec, ball_y_pid*100, 'Color',[1.0 0.4 0.3],'LineWidth',1.8,'DisplayName','Y');
yline(0,'--','Color',[0.7 0.7 0.7],'LineWidth',1.2,'Label','Setpoint');
for d = 1:size(disturb_table,1)
    xline(disturb_table(d,1),'--','Color',[1.0 0.8 0.2],'LineWidth',1.2, ...
        'Label',sprintf('D%d',d),'LabelVerticalAlignment','bottom');
end
ylabel('Ball position [cm]','Color',[0.8 0.8 0.8]);
title('Ball Position (Setpoint = 0)','Color',[0.95 0.95 0.95],'FontWeight','bold');
legend('Location','best','TextColor',[0.8 0.8 0.8],'Color',[0.12 0.12 0.15]);

% Tilt angles
subplot(3,1,2);
set(gca,'Color',[0.08 0.08 0.10],'XColor',[0.8 0.8 0.8],'YColor',[0.8 0.8 0.8],'GridColor',[0.4 0.4 0.4]);
hold on; grid on;
plot(t_vec, pitch_log/deg2rad,'Color',[0.3 0.9 0.4],'LineWidth',1.8,'DisplayName','Pitch cmd');
plot(t_vec, roll_log/deg2rad, 'Color',[1.0 0.8 0.2],'LineWidth',1.8,'DisplayName','Roll cmd');
yline(0,'--','Color',[0.6 0.6 0.6],'LineWidth',1);
for d = 1:size(disturb_table,1)
    xline(disturb_table(d,1),'--','Color',[1.0 0.8 0.2],'LineWidth',1.2);
end
ylabel('PID output [deg]','Color',[0.8 0.8 0.8]);
title('Platform Tilt Commands (PID Output)','Color',[0.95 0.95 0.95],'FontWeight','bold');
legend('Location','best','TextColor',[0.8 0.8 0.8],'Color',[0.12 0.12 0.15]);

% Ball distance from center
subplot(3,1,3);
set(gca,'Color',[0.08 0.08 0.10],'XColor',[0.8 0.8 0.8],'YColor',[0.8 0.8 0.8],'GridColor',[0.4 0.4 0.4]);
hold on; grid on;
dist_pid = sqrt(ball_x_pid.^2 + ball_y_pid.^2) * 100;
plot(t_vec, dist_pid,'Color',[0.9 0.5 1.0],'LineWidth',2.0);
yline(0.5,'--','Color',[0.6 0.6 0.6],'LineWidth',1,'Label','0.5 cm tolerance');
for d = 1:size(disturb_table,1)
    xline(disturb_table(d,1),'--','Color',[1.0 0.8 0.2],'LineWidth',1.2, ...
        'Label',sprintf('D%d',d),'LabelVerticalAlignment','bottom');
end
xlabel('Time [s]','Color',[0.8 0.8 0.8]);
ylabel('Distance [cm]','Color',[0.8 0.8 0.8]);
title('Ball Distance from Center','Color',[0.95 0.95 0.95],'FontWeight','bold');

%% =========================================================
%  PERFORMANCE METRICS CALCULATION
%% =========================================================
fprintf('\n================================================\n');
fprintf('         PID PERFORMANCE METRICS\n');
fprintf('================================================\n');

d0 = dist_pid(1);

% 1. Rise Time (90% to 10% of initial distance)
idx_90 = find(dist_pid <= 0.9*d0, 1, 'first');
idx_10 = find(dist_pid <= 0.1*d0, 1, 'first');
if ~isempty(idx_90) && ~isempty(idx_10)
    rise_time = t_vec(idx_10) - t_vec(idx_90);
    fprintf('Rise Time (90%% -> 10%%) : %.3f s\n', rise_time);
else
    fprintf('Rise Time (90%% -> 10%%) : N/A (Did not reach 10%%)\n');
end

% Find first disturbance time to isolate initial settling
if isempty(disturb_table)
    t_first_dist = t_vec(end);
else
    t_first_dist = disturb_table(1,1);
end
idx_first_dist = find(t_vec >= t_first_dist, 1, 'first');

% 2. Settling Time (to 2% of initial distance)
settle_thresh = 0.02 * d0; 
is_settled = dist_pid(1:idx_first_dist) < settle_thresh;
idx_unsettled = find(~is_settled, 1, 'last');
if isempty(idx_unsettled)
    fprintf('Settling Time (2%%)     : 0.000 s\n');
elseif idx_unsettled < idx_first_dist
    settling_time = t_vec(idx_unsettled + 1);
    fprintf('Settling Time (2%%)     : %.3f s\n', settling_time);
else
    fprintf('Settling Time (2%%)     : N/A (Did not settle before dist.)\n');
end

% 3. Overshoot (Max distance after crossing 10% line, before disturbance)
if ~isempty(idx_10) && idx_10 < idx_first_dist
    max_overshoot = max(dist_pid(idx_10:idx_first_dist));
    overshoot_pct = (max_overshoot / d0) * 100;
    fprintf('Overshoot              : %%%.2f\n', overshoot_pct);
else
    fprintf('Overshoot              : N/A\n');
end

% 4. Disturbance Recovery Time (Average)
rec_thresh = 0.5; % 0.5 cm threshold for recovery
recovery_times = [];
for d = 1:size(disturb_table,1)
    t_d = disturb_table(d,1);
    idx_d = find(t_vec >= t_d, 1, 'first');
    if d < size(disturb_table,1)
        idx_next = find(t_vec >= disturb_table(d+1,1), 1, 'first');
    else
        idx_next = length(t_vec);
    end
    
    window_dist = dist_pid(idx_d:idx_next);
    is_rec = window_dist < rec_thresh;
    idx_unrec = find(~is_rec, 1, 'last');
    
    if ~isempty(idx_unrec) && idx_unrec < length(window_dist)
        t_rec = t_vec(idx_d + idx_unrec) - t_d;
        recovery_times(end+1) = t_rec;
    end
end

if ~isempty(recovery_times)
    fprintf('Avg Dist. Recovery     : %.3f s (Tol: %.1f cm)\n', mean(recovery_times), rec_thresh);
else
    fprintf('Avg Dist. Recovery     : N/A (Did not recover)\n');
end

% 5. Steady-State Error (RMS of last 2 seconds)
idx_last_2s = find(t_vec >= t_vec(end)-2.0, 1, 'first');
if ~isempty(idx_last_2s)
    sse = rms(dist_pid(idx_last_2s:end));
    fprintf('Steady-State Err (RMS) : %.3f cm (calculated over last 2s)\n', sse);
else
    fprintf('Steady-State Err (RMS) : N/A\n');
end

% 6. ITAE Score
fprintf('ITAE Score             : %.3f\n', sim_result.itae);

% 7. Control Effort (RMS of actually-applied tilt after delay)
ctrl_effort = rms(sqrt(sim_result.pitch_act.^2 + sim_result.roll_act.^2)) * (180/pi);
fprintf('Control Effort (RMS)   : %.3f deg\n', ctrl_effort);
fprintf('================================================\n');
