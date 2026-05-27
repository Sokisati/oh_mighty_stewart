% benchmarks/stewart_adaptive_benchmark.m
% Comparative Benchmark for Adaptive PID Controllers
% Compares Classic PID, MRAC Adaptive, and Lyapunov Adaptive against unseen wind seeds.

clc; close all;
fprintf('\n=================================================================\n');
fprintf('   ADAPTIVE PID COMPARATIVE BENCHMARK (Classic vs MRAC vs Lyap)\n');
fprintf('=================================================================\n\n');

run('stewart_setup.m');

%% =========================================================
%  WIND SCENARIOS
%% =========================================================
global WIND_SCENARIOS;
WIND_SCENARIOS = {
    struct('name', 'Scenario 1 (0.8 Chaotic + 0.6 Realistic)', 'type', 'combined', 'c_ratio', 0.8, 'r_ratio', 0.6), ...
    struct('name', 'Scenario 2 (0.4 Chaotic + 0.8 Realistic)', 'type', 'combined', 'c_ratio', 0.4, 'r_ratio', 0.8), ...
    struct('name', 'Scenario 3 (0.0 Chaotic + 1.0 Realistic)', 'type', 'combined', 'c_ratio', 0.0, 'r_ratio', 1.0)
};

num_wind_modes = length(WIND_SCENARIOS);
num_val = 100; % 100 seeds for validation
validation_seeds = randi([20001, 30000], 1, num_val);

config = load_config();
base_Kp = config.Kp;
base_Ki = config.Ki;
base_Kd = config.Kd;

fprintf('[+] Base Analytical PID Gains Loaded from config.txt:\n');
fprintf('    Kp = %.3f, Ki = %.3f, Kd = %.3f\n', base_Kp, base_Ki, base_Kd);
fprintf('[+] Number of Validation Seeds per Scenario: %d\n', num_val);

% Storage for results: 1=Classic, 2=MRAC, 3=Lyapunov
R_all = cell(3, num_wind_modes);
F_all = cell(3, num_wind_modes);
scores_all = cell(3, num_wind_modes);

global WIND_TYPE;
global WIND_C_RATIO;
global WIND_R_RATIO;

for sc = 1:num_wind_modes
    scen = WIND_SCENARIOS{sc};
    fprintf('\n=========================================================================\n');
    fprintf('  RUNNING EXPERIMENT %d/%d: %s\n', sc, num_wind_modes, scen.name);
    fprintf('=========================================================================\n');
    
    WIND_TYPE = scen.type;
    WIND_C_RATIO = scen.c_ratio;
    WIND_R_RATIO = scen.r_ratio;
    
    res1 = zeros(num_val, 6); f1 = zeros(num_val, 1);
    res2 = zeros(num_val, 6); f2 = zeros(num_val, 1);
    res3 = zeros(num_val, 6); f3 = zeros(num_val, 1);
    
    fprintf('Validating %d unseen seeds...\n', num_val);
    h_wait = waitbar(0, sprintf('Scenario %d/%d Validation...', sc, num_wind_modes));
    
    for v = 1:num_val
        seed = validation_seeds(v);
        waitbar(v/num_val, h_wait);
        
        % 1. Classic PID
        [r, f] = run_adaptive_sim('classic', base_Kp, base_Ki, base_Kd, seed, config.dt, config.g_acc, config.c_roll, config.R_base);
        res1(v, :) = r; f1(v) = f;
        
        % 2. MRAC Adaptive PID
        [r, f] = run_adaptive_sim('mrac', base_Kp, base_Ki, base_Kd, seed, config.dt, config.g_acc, config.c_roll, config.R_base);
        res2(v, :) = r; f2(v) = f;
        
        % 3. Lyapunov Adaptive PID
        [r, f] = run_adaptive_sim('lyap', base_Kp, base_Ki, base_Kd, seed, config.dt, config.g_acc, config.c_roll, config.R_base);
        res3(v, :) = r; f3(v) = f;
    end
    close(h_wait);
    
    R_all{1, sc} = res1; F_all{1, sc} = f1;
    R_all{2, sc} = res2; F_all{2, sc} = f2;
    R_all{3, sc} = res3; F_all{3, sc} = f3;
end

%% =========================================================
%  AGGREGATION AND REPORTING
%% =========================================================
w = [0.05, 0.10, 0.20, 0.30, 0.25, 0.10]; 

for sc = 1:num_wind_modes
    scen = WIND_SCENARIOS{sc};
    
    method_names = {
        '1. Classic PID (Fixed)', ...
        '2. MRAC Adaptive PID', ...
        '3. Lyapunov Adaptive PID'
    };
    
    % Reference metrics for score (Classic PID averages)
    R1 = R_all{1, sc}; F1 = F_all{1, sc};
    idx_success = find(F1 == 0);
    if ~isempty(idx_success), a1_ref = mean(R1(idx_success, :), 1); else, a1_ref = mean(R1, 1); end
    
    fprintf('\n=========================================================================================================================================================================\n');
    fprintf('                                              ADAPTIVE BENCHMARK RESULTS FOR %s (%d Seeds)\n', upper(scen.name), num_val);
    fprintf('=========================================================================================================================================================================\n');
    fprintf(' %-38s | Rise Time (s) | Settling (s) | Rejection(cm)| RMS Err (cm) | ITAE Score | Ctrl Effort (deg) | Drops | TOTAL SCORE\n', 'Method');
    fprintf('-------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
    
    for m = 1:3
        R = R_all{m, sc};
        F = F_all{m, sc};
        
        if sum(F == 0) > 0, avg_R = mean(R(F == 0, :), 1); else, avg_R = mean(R, 1); end
        
        scores = zeros(num_val, 1);
        for v = 1:num_val
            if F(v) == 1
                scores(v) = config.drop_penalty;
            else
                scores(v) = 100 * sum(w .* max(0, 2 - (R(v, :) ./ a1_ref)));
            end
        end
        scores_all{m, sc} = scores;
        s_avg = mean(scores);
        
        fprintf(' %-38s | %13.3f | %13.2f | %12.3f | %12.3f | %10.2f | %17.3f | %5d | %11.1f\n', method_names{m}, avg_R(1), avg_R(2), avg_R(3), avg_R(4), avg_R(5), avg_R(6), sum(F), s_avg);
    end
    fprintf('=========================================================================================================================================================================\n\n');
end

% Compute overall averages
fprintf('\n====================================================================================\n');
fprintf('                          FINAL OVERALL SUMMARY (Averaged over %d Wind Modes)\n', num_wind_modes);
fprintf('====================================================================================\n');
fprintf(' %-38s | Overall Average Score\n', 'Method');
fprintf('------------------------------------------------------------------------------------\n');

m_names = {'1. Classic PID (Fixed)', '2. MRAC Adaptive PID', '3. Lyapunov Adaptive PID'};
for m = 1:3
    avg_score = 0;
    for sc = 1:num_wind_modes
        avg_score = avg_score + mean(scores_all{m, sc});
    end
    avg_score = avg_score / num_wind_modes;
    fprintf(' %-38s | %21.2f\n', m_names{m}, avg_score);
end
fprintf('====================================================================================\n\n');

%% =========================================================
%  HEADLESS SIMULATION FUNCTION
%% =========================================================
function [res, fell_off] = run_adaptive_sim(type, Kp_base, Ki_base, Kd_base, seed, dt, g_acc, c_roll, r_limit)
    params.dt       = dt;
    params.T_sim    = 30;
    params.g_acc    = g_acc;
    params.c_roll   = c_roll;
    params.r_limit  = r_limit;
    params.max_tilt = 30 * (pi/180);
    params.ball_x0  = 0.05;
    params.ball_y0  = 0.03;

    if strcmp(type, 'mrac')
        params.mrac.active = true;
        params.mrac.gamma_p = 1500.0;
        params.mrac.sigma_p = 4.0;
        params.mrac.gamma_i = 100.0;
        params.mrac.sigma_i = 2.0;
        params.mrac.gamma_d = 20.0;
        params.mrac.sigma_d = 5.0;
    elseif strcmp(type, 'lyap')
        params.lyap.active = true;
        params.lyap.lambda = 5.0;
        params.lyap.gamma_p = 500.0;
        params.lyap.sigma_p = 2.0;
        params.lyap.gamma_i = 200.0;
        params.lyap.sigma_i = 1.0;
        params.lyap.gamma_d = 50.0;
        params.lyap.sigma_d = 5.0;
    end

    disturb_table = generate_disturbances(seed, params.T_sim);
    noise_table   = generate_sensor_noise(seed, params.T_sim, dt);

    result = simulate_ball(Kp_base, Ki_base, Kd_base, params, disturb_table, noise_table);
    fell_off = result.fell_off;

    ball_x = result.ball_x;
    ball_y = result.ball_y;
    t_vec  = result.t_vec;

    dist_cm = sqrt(ball_x.^2 + ball_y.^2) * 100;
    d0 = dist_cm(1);
    if d0 == 0, d0 = 0.001; end

    idx_90 = find(dist_cm <= 0.9*d0, 1, 'first');
    idx_10 = find(dist_cm <= 0.1*d0, 1, 'first');
    if ~isempty(idx_90) && ~isempty(idx_10)
        rise_time = t_vec(idx_10) - t_vec(idx_90);
    else
        rise_time = params.T_sim; 
    end

    idx_settled = find(dist_cm <= 1.0, 1, 'first');
    if ~isempty(idx_settled)
        settling_time = t_vec(idx_settled);
    else
        settling_time = params.T_sim;
    end

    if isempty(disturb_table)
        t_wind_start = t_vec(end);
    else
        t_wind_start = disturb_table(1,1);
    end
    idx_wind_eval = find(t_vec >= (t_wind_start + 1.0), 1, 'first');
    if isempty(idx_wind_eval), idx_wind_eval = 1; end
    wind_rejection = rms(dist_cm(idx_wind_eval:end));

    idx_last_2s = find(t_vec >= t_vec(end)-2.0, 1, 'first');
    if isempty(idx_last_2s), idx_last_2s = 1; end
    rms_err = rms(dist_cm(idx_last_2s:end));

    itae = result.itae;
    ctrl_effort = rms(sqrt(result.pitch_act.^2 + result.roll_act.^2)) * (180/pi);

    res = [rise_time, settling_time, wind_rejection, rms_err, itae, ctrl_effort];
end
