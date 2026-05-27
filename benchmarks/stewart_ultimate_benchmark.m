
clc; close all;
fprintf('\n====================================================\n');
fprintf('               ULTIMATE BENCHMARK\n');
fprintf('====================================================\n\n');

run('stewart_setup.m');

global WIND_SCENARIOS;
WIND_SCENARIOS = {
    struct('name', 'Scenario 1 (0.8 Chaotic + 0.6 Realistic)', 'type', 'combined', 'c_ratio', 0.8, 'r_ratio', 0.6), ...
    struct('name', 'Scenario 2 (0.4 Chaotic + 0.8 Realistic)', 'type', 'combined', 'c_ratio', 0.4, 'r_ratio', 0.8), ...
    struct('name', 'Scenario 3 (0.0 Chaotic + 1.0 Realistic)', 'type', 'combined', 'c_ratio', 0.0, 'r_ratio', 1.0)
};

num_wind_modes = length(WIND_SCENARIOS);
num_val = 100; % 100 seeds for validation
validation_seeds = randi([40001, 50000], 1, num_val);

config = load_config();
base_Kp = config.Kp;
base_Ki = config.Ki;
base_Kd = config.Kd;

R_all = cell(4, num_wind_modes);
F_all = cell(4, num_wind_modes);
scores_all = cell(4, num_wind_modes);
Params = cell(2, num_wind_modes); % Store Analytic vs GA params

global WIND_TYPE;
global WIND_C_RATIO;
global WIND_R_RATIO;

nlpid_logs_all = cell(num_wind_modes, 1);

for sc = 1:num_wind_modes
    scen = WIND_SCENARIOS{sc};
    fprintf('\n=========================================================================\n');
    fprintf('  RUNNING EXPERIMENT %d/%d: %s\n', sc, num_wind_modes, scen.name);
    fprintf('=========================================================================\n\n');
    
    WIND_TYPE = scen.type;
    WIND_C_RATIO = scen.c_ratio;
    WIND_R_RATIO = scen.r_ratio;
    
    Params{1, sc} = [base_Kp, base_Ki, base_Kd];
    
    fprintf('--- TRAINING PHASE ---\n');
    fprintf('[GA] Training Island Genetic Algorithm to find optimal baseline...\n');
    [Kp_ga, Ki_ga, Kd_ga] = stewart_ga_multi_seed(50, config.ga_pop_size, config.ga_generations, false, 42);
    Params{2, sc} = [Kp_ga, Ki_ga, Kd_ga];
    
    fprintf('\n--- VALIDATION PHASE (%d Seeds) ---\n', num_val);
    
    for m = 1:4
        R_all{m, sc} = zeros(num_val, 6);
        F_all{m, sc} = zeros(num_val, 1);
    end
    
    for v = 1:num_val
        vs = validation_seeds(v);
        if mod(v, 10) == 0
            fprintf('.');
        end
        
        p = Params{1, sc};
        [r, f, ~] = run_adaptive_sim('classic', p(1), p(2), p(3), vs, config.dt, config.g_acc, config.c_roll, config.R_base);
        R_all{1, sc}(v,:) = r; F_all{1, sc}(v) = f;
        
        [r, f, ~] = run_adaptive_sim('nlpid', p(1), p(2), p(3), vs, config.dt, config.g_acc, config.c_roll, config.R_base);
        R_all{2, sc}(v,:) = r; F_all{2, sc}(v) = f;
        
        p_ga = Params{2, sc};
        [r, f, ~] = run_adaptive_sim('classic', p_ga(1), p_ga(2), p_ga(3), vs, config.dt, config.g_acc, config.c_roll, config.R_base);
        R_all{3, sc}(v,:) = r; F_all{3, sc}(v) = f;
        
        [r, f, n_logs] = run_adaptive_sim('nlpid', p_ga(1), p_ga(2), p_ga(3), vs, config.dt, config.g_acc, config.c_roll, config.R_base);
        R_all{4, sc}(v,:) = r; F_all{4, sc}(v) = f;
        
        if v == 1
            nlpid_logs_all{sc} = n_logs;
        end
    end
    fprintf('\n');
end

w = [0.05, 0.10, 0.20, 0.30, 0.25, 0.10]; 

for sc = 1:num_wind_modes
    scen = WIND_SCENARIOS{sc};
    
    method_names = {
        sprintf('1. Classic PID (%.2f, %.2f, %.2f)', Params{1, sc}(1), Params{1, sc}(2), Params{1, sc}(3)), ...
        sprintf('2. NLPID Adaptive (%.2f, %.2f, %.2f)', Params{1, sc}(1), Params{1, sc}(2), Params{1, sc}(3)), ...
        sprintf('3. Genetic PID (%.2f, %.2f, %.2f)', Params{2, sc}(1), Params{2, sc}(2), Params{2, sc}(3)), ...
        sprintf('4. Gen-Adapt NLPID (%.2f, %.2f, %.2f)', Params{2, sc}(1), Params{2, sc}(2), Params{2, sc}(3))
    };
    
    R1 = R_all{1, sc}; F1 = F_all{1, sc};
    idx_success = find(F1 == 0);
    if ~isempty(idx_success), a1_ref = mean(R1(idx_success, :), 1); else, a1_ref = mean(R1, 1); end
    
    fprintf('\n=========================================================================================================================================================================\n');
    fprintf('                                              ULTIMATE BENCHMARK RESULTS FOR %s (%d Seeds)\n', upper(scen.name), num_val);
    fprintf('=========================================================================================================================================================================\n');
    fprintf(' %-38s | Rise Time (s) | Settling (s) | Rejection(cm)| RMS Err (cm) | ITAE Score | Ctrl Effort (deg) | Drops | TOTAL SCORE\n', 'Method');
    fprintf('-------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
    
    for m = 1:4
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

fprintf('\n====================================================================================\n');
fprintf('                          FINAL OVERALL SUMMARY (Averaged over %d Wind Modes)\n', num_wind_modes);
fprintf('====================================================================================\n');
fprintf(' %-38s | Overall Average Score\n', 'Method');
fprintf('------------------------------------------------------------------------------------\n');

m_names = {'1. Classic PID (Analytic Fixed)', '2. NLPID Adaptive (Analytic Base)', '3. Genetic PID (GA Fixed)', '4. Genetic Adaptive (GA Base)'};
for m = 1:4
    avg_score = 0;
    for sc = 1:num_wind_modes
        avg_score = avg_score + mean(scores_all{m, sc});
    end
    avg_score = avg_score / num_wind_modes;
    fprintf(' %-38s | %21.2f\n', m_names{m}, avg_score);
end
fprintf('====================================================================================\n\n');

figure('Name', 'Genetic Adaptive NLPID Gain Histories (Seed #1)', 'Position', [100, 100, 1200, 800]);
for sc = 1:num_wind_modes
    n_logs = nlpid_logs_all{sc};
    t = n_logs.t;
    
    base_Kp_ga = Params{2, sc}(1);
    base_Ki_ga = Params{2, sc}(2);
    base_Kd_ga = Params{2, sc}(3);
    
    subplot(3, 1, sc);
    plot(t, n_logs.Kp, 'r', 'LineWidth', 1.5); hold on;
    plot(t, n_logs.Ki, 'g', 'LineWidth', 1.5);
    plot(t, n_logs.Kd, 'b', 'LineWidth', 1.5);
    
    plot([t(1) t(end)], [base_Kp_ga base_Kp_ga], 'r--');
    plot([t(1) t(end)], [base_Ki_ga base_Ki_ga], 'g--');
    plot([t(1) t(end)], [base_Kd_ga base_Kd_ga], 'b--');
    hold off;
    
    title(sprintf('GA-NLPID Gain Adaptation - %s', WIND_SCENARIOS{sc}.name));
    xlabel('Time (s)');
    ylabel('Gain Value');
    legend('Kp', 'Ki', 'Kd', 'Location', 'best');
    grid on;
end

function [res, fell_off, logs] = run_adaptive_sim(type, Kp_base, Ki_base, Kd_base, seed, dt, g_acc, c_roll, r_limit)
    params.dt       = dt;
    params.T_sim    = 30;
    params.g_acc    = g_acc;
    params.c_roll   = c_roll;
    params.r_limit  = r_limit;
    params.max_tilt = 30 * (pi/180);
    params.ball_x0  = 0.05;
    params.ball_y0  = 0.03;

    if strcmp(type, 'nlpid')
        params.nlpid.active = true;
        params.nlpid.e_scale = 0.03;   % 3cm is the saturation scale for error
        params.nlpid.i_scale = 0.02;   % 2cm.s is the saturation scale for integral
        params.nlpid.kp_boost = 1.5;   % Max Kp boost
        params.nlpid.kd_boost = 0.5;   % Max Kd boost
        params.nlpid.ki_boost = 1.5;   % Max Ki boost
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
    
    logs.t = t_vec;
    logs.Kp = result.Kp_log;
    logs.Ki = result.Ki_log;
    logs.Kd = result.Kd_log;
end
