% benchmarks/stewart_adaptive_benchmark.m
% Comparative benchmark between Classic PID, MRAC, and Lyapunov Adaptive PID
% using the SAME base Analytical PID values.

clc; close all;

fprintf('=================================================================\n');
fprintf('       ADAPTIVE PID COMPARATIVE BENCHMARK (Mode 8)\n');
fprintf('=================================================================\n\n');

% 1. Setup
this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(this_dir, '..', 'core')));
addpath(genpath(fullfile(this_dir, '..', 'environment')));

conf = load_config(); % Loads Kp, Ki, Kd and physics

fprintf('[+] Base Analytical PID Gains Loaded from config.txt:\n');
fprintf('    Kp = %.3f, Ki = %.3f, Kd = %.3f\n\n', conf.Kp, conf.Ki, conf.Kd);

% Common params
params.dt       = conf.dt;
params.T_sim    = 20.0; % 20 seconds is enough for benchmark
params.g_acc    = conf.g_acc;
params.c_roll   = conf.c_roll;
params.r_limit  = conf.R_base;
params.max_tilt = conf.max_tilt_deg * (pi/180);
params.delay_sec = conf.delay_sec;
params.slew_rate = conf.slew_rate_deg * (pi/180);
params.ball_x0  = 0.0;
params.ball_y0  = 0.0;

% Force combined wind for a tough scenario
global WIND_TYPE; WIND_TYPE = 'combined';
global WIND_C_RATIO; WIND_C_RATIO = 0.5;
global WIND_R_RATIO; WIND_R_RATIO = 1.0;

% Generate shared disturbance
seed = 42; % Fixed seed for fair comparison
disturb_table = generate_disturbances(seed, params.T_sim);
noise_table   = generate_sensor_noise(seed, params.T_sim, params.dt);

fprintf('[+] Shared Disturbance (Seed %d) and Sensor Noise generated.\n\n', seed);

% --- SIMULATION 1: Classic PID ---
fprintf('--- SIMULATION 1/3: Classic PID ---\n');
p1 = params;
res_classic = simulate_ball(conf.Kp, conf.Ki, conf.Kd, p1, disturb_table, noise_table);
fprintf('    ITAE Score: %.2f | Drops: %d\n\n', res_classic.itae, res_classic.fell_off);

% --- SIMULATION 2: MRAC Adaptive PID ---
fprintf('--- SIMULATION 2/3: MRAC Adaptive PID ---\n');
p2 = params;
p2.mrac.active = true;
p2.mrac.gamma_p = 1500.0;
p2.mrac.sigma_p = 4.0;
p2.mrac.gamma_i = 100.0;
p2.mrac.sigma_i = 2.0;
p2.mrac.gamma_d = 20.0;
p2.mrac.sigma_d = 5.0;
res_mrac = simulate_ball(conf.Kp, conf.Ki, conf.Kd, p2, disturb_table, noise_table);
fprintf('    ITAE Score: %.2f | Drops: %d\n\n', res_mrac.itae, res_mrac.fell_off);

% --- SIMULATION 3: Lyapunov Adaptive PID ---
fprintf('--- SIMULATION 3/3: Lyapunov Adaptive PID ---\n');
p3 = params;
p3.lyap.active = true;
p3.lyap.lambda = 5.0;
p3.lyap.gamma_p = 500.0;
p3.lyap.sigma_p = 2.0;
p3.lyap.gamma_i = 200.0;
p3.lyap.sigma_i = 1.0;
p3.lyap.gamma_d = 50.0;
p3.lyap.sigma_d = 5.0;
res_lyap = simulate_ball(conf.Kp, conf.Ki, conf.Kd, p3, disturb_table, noise_table);
fprintf('    ITAE Score: %.2f | Drops: %d\n\n', res_lyap.itae, res_lyap.fell_off);

%% PLOTTING RESULTS
fprintf('[+] Generating Comparative Plots...\n');
fig = figure('Name', 'Adaptive PID Comparison', 'Position', [150, 100, 1000, 700], 'Color', 'w');

% Plot 1: Radial Distance
subplot(2, 1, 1);
hold on; grid on;
r_classic = sqrt(res_classic.ball_x.^2 + res_classic.ball_y.^2) * 100;
r_mrac    = sqrt(res_mrac.ball_x.^2 + res_mrac.ball_y.^2) * 100;
r_lyap    = sqrt(res_lyap.ball_x.^2 + res_lyap.ball_y.^2) * 100;

plot(res_classic.t_vec, r_classic, 'k-', 'LineWidth', 1.5, 'DisplayName', sprintf('Classic PID (ITAE: %.1f)', res_classic.itae));
plot(res_mrac.t_vec, r_mrac, 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('MRAC Adaptive (ITAE: %.1f)', res_mrac.itae));
plot(res_lyap.t_vec, r_lyap, 'r-', 'LineWidth', 1.5, 'DisplayName', sprintf('Lyapunov Adaptive (ITAE: %.1f)', res_lyap.itae));

yline(params.r_limit * 100, 'r--', 'Platform Edge', 'LineWidth', 2, 'HandleVisibility','off');
xlabel('Zaman (s)');
ylabel('Merkeze Uzaklık (cm)');
title('Topun Merkezden Sapması (Zorlu Rüzgar & Gürültü Altında)');
legend('Location', 'best');
set(gca, 'FontSize', 11);

% Plot 2: Adaptive Kp Evolution
subplot(2, 1, 2);
hold on; grid on;
plot(res_classic.t_vec, res_classic.Kp_log, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Classic Kp (Fixed)');
plot(res_mrac.t_vec, res_mrac.Kp_log, 'b-', 'LineWidth', 1.5, 'DisplayName', 'MRAC Kp');
plot(res_lyap.t_vec, res_lyap.Kp_log, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Lyapunov Kp');

xlabel('Zaman (s)');
ylabel('Kp Kazancı (Proportional Gain)');
title('Adaptif Kontrolcülerin Kp Kazancını Dinamik Olarak Ayarlaması');
legend('Location', 'best');
set(gca, 'FontSize', 11);

sgtitle('Classic PID vs MRAC vs Lyapunov Adaptive Control', 'FontWeight', 'bold', 'FontSize', 14);

fprintf('=================================================================\n');
fprintf('  SONUÇ ÖZETİ (Daha düşük ITAE = Daha iyi performans):\n');
fprintf('  Classic PID ITAE  : %8.2f\n', res_classic.itae);
fprintf('  MRAC PID ITAE     : %8.2f\n', res_mrac.itae);
fprintf('  Lyapunov PID ITAE : %8.2f\n', res_lyap.itae);
fprintf('=================================================================\n');
