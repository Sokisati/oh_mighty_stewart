
clc; close all;
fprintf('\n====================================================\n');
fprintf('   EVOLUTIONARY ALGORITHMS COMPARATIVE BENCHMARK\n');
fprintf('====================================================\n\n');

run('stewart_setup.m');

config = load_config();

global WIND_SCENARIOS;
WIND_SCENARIOS = {
    struct('name', sprintf('Scenario 1 (%.1f Chaotic + %.1f Realistic)', config.bench_scen1_c, config.bench_scen1_r), 'type', 'combined', 'c_ratio', config.bench_scen1_c, 'r_ratio', config.bench_scen1_r), ...
    struct('name', sprintf('Scenario 2 (%.1f Chaotic + %.1f Realistic)', config.bench_scen2_c, config.bench_scen2_r), 'type', 'combined', 'c_ratio', config.bench_scen2_c, 'r_ratio', config.bench_scen2_r), ...
    struct('name', sprintf('Scenario 3 (%.1f Chaotic + %.1f Realistic)', config.bench_scen3_c, config.bench_scen3_r), 'type', 'combined', 'c_ratio', config.bench_scen3_c, 'r_ratio', config.bench_scen3_r)
};

num_wind_modes = length(WIND_SCENARIOS);
num_val = config.bench_comparative_seeds; % seeds for validation
validation_seeds = randi([20001, 30000], 1, num_val);
base_Kp = config.Kp;
base_Ki = config.Ki;
base_Kd = config.Kd;

R_all = cell(3, num_wind_modes);
F_all = cell(3, num_wind_modes);
scores_all = cell(3, num_wind_modes);
Params = cell(3, num_wind_modes); 

global WIND_TYPE;
global WIND_C_RATIO;
global WIND_R_RATIO;

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
    fprintf('[GA] Training Robust Island Genetic Algorithm...\n');
    [Kp_ga, Ki_ga, Kd_ga] = stewart_ga_multi_seed(50, config.ga_pop_size, config.ga_generations, false, 42);
    Params{2, sc} = [Kp_ga, Ki_ga, Kd_ga];
    
    fprintf('[CMA-ES] Training Robust IPOP-CMA-ES...\n');
    [Kp_cmaes, Ki_cmaes, Kd_cmaes] = stewart_cmaes_multi_seed(50, config.cmaes_lambda, config.cmaes_generations, false, 42);
    Params{3, sc} = [Kp_cmaes, Ki_cmaes, Kd_cmaes];
    
    fprintf('\n--- VALIDATION PHASE (%d Seeds) ---\n', num_val);
    
    for m = 1:3
        R_all{m, sc} = zeros(num_val, 6);
        F_all{m, sc} = zeros(num_val, 1);
    end
    
    for v = 1:num_val
        vs = validation_seeds(v);
        
        for m = 1:3
            p = Params{m, sc};
            [res, fell] = run_headless_sim(p(1), p(2), p(3), vs, dt, g_acc, c_roll, r_limit);
            R_all{m, sc}(v,:) = res;
            F_all{m, sc}(v) = fell;
        end
    end
end

w = [0.05, 0.10, 0.20, 0.30, 0.25, 0.10]; 

for sc = 1:num_wind_modes
    scen = WIND_SCENARIOS{sc};
    
    method_names = {
        sprintf('1. Classic PID (%.2f, %.2f, %.2f)', Params{1, sc}), ...
        sprintf('2. Robust GA PID (%.2f, %.2f, %.2f)', Params{2, sc}), ...
        sprintf('3. Robust CMA-ES (%.2f, %.2f, %.2f)', Params{3, sc})
    };
    
    R1 = R_all{1, sc}; F1 = F_all{1, sc};
    idx_success = find(F1 == 0);
    if ~isempty(idx_success), a1_ref = mean(R1(idx_success, :), 1); else, a1_ref = mean(R1, 1); end
    
    fprintf('\n=========================================================================================================================================================================\n');
    fprintf('                                              COMPARATIVE BENCHMARK RESULTS FOR %s (50 Seeds)\n', upper(scen.name));
    fprintf('=========================================================================================================================================================================\n');
    fprintf(' %-38s | Rise Time (s) | Settling (s) | Rejection(cm)| RMS Err (cm) | ITAE Score | Ctrl Effort (deg) | Drops | TOTAL SCORE\n', 'Method (Kp, Ki, Kd)');
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

fprintf('\n====================================================================================\n');
fprintf('                          FINAL OVERALL SUMMARY (Averaged over %d Wind Modes)\n', num_wind_modes);
fprintf('====================================================================================\n');
fprintf(' %-38s | Overall Average Score\n', 'Method');
fprintf('------------------------------------------------------------------------------------\n');

m_names = {'1. Classic PID', '2. Robust GA PID', '3. Robust CMA-ES'};
for m = 1:3
    avg_score = 0;
    for sc = 1:num_wind_modes
        avg_score = avg_score + mean(scores_all{m, sc});
    end
    avg_score = avg_score / num_wind_modes;
    fprintf(' %-38s | %21.2f\n', m_names{m}, avg_score);
end
fprintf('====================================================================================\n\n');

function [res, fell_off] = run_headless_sim(Kp_base, Ki_base, Kd_base, seed, dt, g_acc, c_roll, r_limit)
    params.dt       = dt;
    params.T_sim    = 30;
    params.g_acc    = g_acc;
    params.c_roll   = c_roll;
    params.r_limit  = r_limit;
    params.max_tilt = 30 * (pi/180);
    params.ball_x0  = 0.05;
    params.ball_y0  = 0.03;

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
