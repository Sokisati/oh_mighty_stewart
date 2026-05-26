%% stewart_final_benchmark.m
%  The Ultimate Benchmark Suite
%  Compares 4 control strategies on 10 unseen wind seeds.
%  Uses the EXACT same physics loop as stewart_pid_sim.m (no interp1).
%
%  Methods:
%    1. Classic PID        (Delay-tuned: Kp=10, Ki=0.5, Kd=2.0, fixed)
%    2. Gain Scheduling    (Same base + error-driven adaptive scaling)
%    3. Genetic Algorithm  (GA-tuned for 60ms actuator delay)
%    4. GS + GA Hybrid     (GA-tuned base + adaptive scaling)

clc; close all;
fprintf('\n====================================================\n');
fprintf('     THE ULTIMATE BENCHMARK SUITE (MULTI-WIND)\n');
fprintf('====================================================\n\n');

run('stewart_setup.m');

%% =========================================================
%  MODULAR WIND SCENARIOS DEFINITION
%  Easily add or modify coefficients here. Benchmark-specific.
%% =========================================================
global WIND_SCENARIOS;
WIND_SCENARIOS = {
    struct('name', 'Scenario 1 (0.9 Chaotic + 0.5 Realistic)', 'type', 'combined', 'c_ratio', 0.9, 'r_ratio', 0.5), ...
    struct('name', 'Scenario 2 (0.4 Chaotic + 0.8 Realistic)', 'type', 'combined', 'c_ratio', 0.4, 'r_ratio', 0.8), ...
    struct('name', 'Scenario 3 (0 Chaotic + 1.0 Realistic)', 'type', 'combined', 'c_ratio', 0, 'r_ratio', 1.0)
};

num_wind_modes = length(WIND_SCENARIOS);

%% =========================================================
%  PHASE 2 & 3: VALIDATION ON 100 NEW SEEDS FOR EACH WIND SCENARIO
%% =========================================================
num_val = 100;
validation_seeds = randi([20001, 30000], 1, num_val);

% Base PID parameters (Loaded dynamically from central config.txt)
config = load_config();
base_Kp = config.Kp;
base_Ki = config.Ki;
base_Kd = config.Kd;

% Performance logs for each scenario
R1_all = cell(num_wind_modes, 1);
R2_all = cell(num_wind_modes, 1);
F1_all = cell(num_wind_modes, 1);
F2_all = cell(num_wind_modes, 1);
scores1_all = cell(num_wind_modes, 1);
scores2_all = cell(num_wind_modes, 1);
GA_params = cell(num_wind_modes, 1); % To store the specific Kp, Ki, Kd found for each wind scenario

global WIND_TYPE;
global WIND_C_RATIO;
global WIND_R_RATIO;

for sc = 1:num_wind_modes
    scen = WIND_SCENARIOS{sc};
    fprintf('\n=========================================================================\n');
    fprintf('  RUNNING EXPERIMENT %d/%d: %s\n', sc, num_wind_modes, scen.name);
    fprintf('=========================================================================\n\n');
    
    % Apply active wind parameters globally so GA and validation runs use ONLY this scenario's wind
    WIND_TYPE = scen.type;
    WIND_C_RATIO = scen.c_ratio;
    WIND_R_RATIO = scen.r_ratio;
    
    % --- GA TRAINING FOR THIS WIND SCENARIO ---
    fprintf('[GA TRAINING] Training Robust Genetic Algorithm specifically for this wind mode...\n');
    [GA_Kp, GA_Ki, GA_Kd] = stewart_ga_multi_seed(100, 30, 50, false, 42);
    GA_params{sc} = [GA_Kp, GA_Ki, GA_Kd];
    fprintf('\n[GA TRAINING COMPLETE] Found Kp: %.3f, Ki: %.3f, Kd: %.3f\n\n', GA_Kp, GA_Ki, GA_Kd);
    
    % --- VALIDATION FOR THIS WIND SCENARIO ---
    fprintf('[VALIDATION] Evaluating Classic PID vs GA PID on %d unseen seeds...\n', num_val);
    R1 = zeros(num_val, 6); % Classic PID
    R2 = zeros(num_val, 6); % Robust GA PID
    F1 = zeros(num_val, 1);
    F2 = zeros(num_val, 1);
    
    for v = 1:num_val
        vs = validation_seeds(v);
        
        % Method 1: Classic PID (Uses params exactly from stewart_pid_sim.m)
        [res1, fell1] = run_headless_sim(base_Kp, base_Ki, base_Kd, vs, dt, g_acc, c_roll, r_limit);
        R1(v,:) = res1; F1(v) = fell1;
        
        % Method 2: Robust GA PID (tuned specifically for this scenario!)
        [res2, fell2] = run_headless_sim(GA_Kp, GA_Ki, GA_Kd, vs, dt, g_acc, c_roll, r_limit);
        R2(v,:) = res2; F2(v) = fell2;
    end
    
    R1_all{sc} = R1;
    R2_all{sc} = R2;
    F1_all{sc} = F1;
    F2_all{sc} = F2;
end

%% =========================================================
%  PHASE 4: AGGREGATION AND REPORTING
%% =========================================================
w = [0.05, 0.10, 0.20, 0.30, 0.25, 0.10]; % Weights: Rise(5%), Settling(10%), Rejection(20%), RMS(30%), ITAE(25%), Effort(10%)
str1 = sprintf('1. Classic PID (%.1f, %.1f, %.1f)', base_Kp, base_Ki, base_Kd);

for sc = 1:num_wind_modes
    scen = WIND_SCENARIOS{sc};
    R1 = R1_all{sc};
    R2 = R2_all{sc};
    F1 = F1_all{sc};
    F2 = F2_all{sc};
    
    GA_Kp = GA_params{sc}(1);
    GA_Ki = GA_params{sc}(2);
    GA_Kd = GA_params{sc}(3);
    str2 = sprintf('2. Robust GA PID (%.1f, %.1f, %.1f)', GA_Kp, GA_Ki, GA_Kd);
    
    % Successful run averages
    if sum(F1 == 0) > 0, a1 = mean(R1(F1 == 0, :), 1); else, a1 = mean(R1, 1); end
    if sum(F2 == 0) > 0, a2 = mean(R2(F2 == 0, :), 1); else, a2 = mean(R2, 1); end
    
    % Reference metrics for score (Classic PID averages in this specific scenario)
    idx_success_pid = find(F1 == 0);
    if ~isempty(idx_success_pid), a1_ref = mean(R1(idx_success_pid, :), 1); else, a1_ref = mean(R1, 1); end
    
    scores1 = zeros(num_val, 1);
    scores2 = zeros(num_val, 1);
    
    for v = 1:num_val
        if F1(v) == 1
            scores1(v) = config.drop_penalty;
        else
            scores1(v) = 100 * sum(w .* max(0, 2 - (R1(v, :) ./ a1_ref)));
        end
        
        if F2(v) == 1
            scores2(v) = config.drop_penalty;
        else
            scores2(v) = 100 * sum(w .* max(0, 2 - (R2(v, :) ./ a1_ref)));
        end
    end
    
    scores1_all{sc} = scores1;
    scores2_all{sc} = scores2;
    
    s1_avg = mean(scores1);
    s2_avg = mean(scores2);
    
    fprintf('\n=========================================================================================================================================================================\n');
    fprintf('                                                  BENCHMARK RESULTS FOR %s (100 Seeds)\n', upper(scen.name));
    fprintf('=========================================================================================================================================================================\n');
    fprintf(' %-38s | Rise Time (s) | Settling (s) | Rejection(cm)| RMS Err (cm) | ITAE Score | Ctrl Effort (deg) | Drops | TOTAL SCORE\n', 'Method (Kp, Ki, Kd)');
    fprintf('-------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
    fprintf(' %-38s | %13.3f | %13.2f | %12.3f | %12.3f | %10.2f | %17.3f | %5d | %11.1f\n', str1, a1(1), a1(2), a1(3), a1(4), a1(5), a1(6), sum(F1), s1_avg);
    fprintf(' %-38s | %13.3f | %13.2f | %12.3f | %12.3f | %10.2f | %17.3f | %5d | %11.1f\n', str2, a2(1), a2(2), a2(3), a2(4), a2(5), a2(6), sum(F2), s2_avg);
    fprintf('=========================================================================================================================================================================\n\n');
end

% Compute overall averages
avg_score1 = 0;
avg_score2 = 0;
for sc = 1:num_wind_modes
    avg_score1 = avg_score1 + mean(scores1_all{sc});
    avg_score2 = avg_score2 + mean(scores2_all{sc});
end
avg_score1 = avg_score1 / num_wind_modes;
avg_score2 = avg_score2 / num_wind_modes;

fprintf('\n====================================================================================\n');
fprintf('                          FINAL OVERALL SUMMARY (Averaged over %d Wind Modes)\n', num_wind_modes);
fprintf('====================================================================================\n');
fprintf(' %-38s | Overall Average Score\n', 'Method (Kp, Ki, Kd)');
fprintf('------------------------------------------------------------------------------------\n');
fprintf(' %-38s | %21.2f\n', str1, avg_score1);
fprintf(' %-38s | Overall Average GA Score: %6.1f\n', '2. Robust GA PID (Scenario-Specific)', avg_score2);
fprintf('====================================================================================\n\n');


%% =========================================================
%  HEADLESS SIMULATION FUNCTION
%  Replicates the EXACT physics loop from stewart_pid_sim.m
%  with NO animation, NO interp1. Impulse kicks only.
%% =========================================================
function [res, fell_off] = run_headless_sim(Kp_base, Ki_base, Kd_base, seed, dt, g_acc, c_roll, r_limit)
    % --- Build params struct for simulate_ball ---
    params.dt       = dt;
    params.T_sim    = 30;
    params.g_acc    = g_acc;
    params.c_roll   = c_roll;
    params.r_limit  = r_limit;
    params.max_tilt = 30 * (pi/180);
    params.ball_x0  = 0.05;
    params.ball_y0  = 0.03;

    % --- Generate disturbances and noise ---
    disturb_table = generate_disturbances(seed, params.T_sim);
    noise_table   = generate_sensor_noise(seed, params.T_sim, dt);

    % --- Run unified physics engine ---
    result = simulate_ball(Kp_base, Ki_base, Kd_base, params, disturb_table, noise_table);
    fell_off = result.fell_off;

    % --- Extract arrays from result ---
    ball_x = result.ball_x;
    ball_y = result.ball_y;
    t_vec  = result.t_vec;

    % === BENCHMARK-SPECIFIC METRICS ===
    dist_cm = sqrt(ball_x.^2 + ball_y.^2) * 100;
    d0 = dist_cm(1);
    if d0 == 0, d0 = 0.001; end % Prevent division by zero

    % 1. Rise Time
    idx_90 = find(dist_cm <= 0.9*d0, 1, 'first');
    idx_10 = find(dist_cm <= 0.1*d0, 1, 'first');
    if ~isempty(idx_90) && ~isempty(idx_10)
        rise_time = t_vec(idx_10) - t_vec(idx_90);
    else
        rise_time = params.T_sim; % Penalty for never reaching 10%
    end

    % 2. Settling Time (s)
    idx_settled = find(dist_cm <= 1.0, 1, 'first');
    if ~isempty(idx_settled)
        settling_time = t_vec(idx_settled);
    else
        settling_time = params.T_sim;
    end

    % 3. Wind Rejection RMS (cm)
    if isempty(disturb_table)
        t_wind_start = t_vec(end);
    else
        t_wind_start = disturb_table(1,1);
    end
    idx_wind_eval = find(t_vec >= (t_wind_start + 1.0), 1, 'first');
    if isempty(idx_wind_eval)
        idx_wind_eval = 1;
    end
    wind_rejection = rms(dist_cm(idx_wind_eval:end));

    % 4. RMS Error (last 2 seconds)
    idx_last_2s = find(t_vec >= t_vec(end)-2.0, 1, 'first');
    if isempty(idx_last_2s)
        idx_last_2s = 1;
    end
    rms_err = rms(dist_cm(idx_last_2s:end));

    % 5. ITAE Score
    itae = result.itae;

    % 6. Control Effort (RMS of actually-applied tilt in degrees, after delay)
    ctrl_effort = rms(sqrt(result.pitch_act.^2 + result.roll_act.^2)) * (180/pi);

    res = [rise_time, settling_time, wind_rejection, rms_err, itae, ctrl_effort];
end

