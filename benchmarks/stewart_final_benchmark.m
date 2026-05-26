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
fprintf('     THE ULTIMATE BENCHMARK SUITE\n');
fprintf('====================================================\n\n');

run('stewart_setup.m');

%% =========================================================
%  PHASE 1: OFFLINE GA TRAINING (30 SEEDS)
%  =========================================================
fprintf('[PHASE 1] Training Robust Genetic Algorithm...\n');
fprintf('          (Using the unified stewart_ga_multi_seed function)\n');

% Train robust GA over 30 scenarios, population of 30, for 15 generations, without live plotting
[GA_Kp, GA_Ki, GA_Kd] = stewart_ga_multi_seed(30, 30, 15, false);

fprintf('\n[PHASE 1 COMPLETE]\n');
fprintf('  -> Robust Tuned GA_Kp: %.3f\n', GA_Kp);
fprintf('  -> Robust Tuned GA_Ki: %.3f\n', GA_Ki);
fprintf('  -> Robust Tuned GA_Kd: %.3f\n\n', GA_Kd);

%% =========================================================
%  PHASE 2 & 3: VALIDATION ON 100 NEW SEEDS
%  =========================================================
num_val = 100;
validation_seeds = randi([20001, 30000], 1, num_val);

fprintf('[PHASE 2] Validating 4 algorithms on %d unseen wind seeds...\n', num_val);

% Gain Scheduling parameters
% With 60ms delay, Kp_eff = Kp_base + alpha_p * err must stay stable.
% At max typical err=5cm, alpha_p=40 gives Kp_eff=10+40*0.05=12 (safe margin)
alpha_p = 40.0;
alpha_d = 30.0;

% Storage: [rise_time, overshoot, avg_recovery, rms_err, itae, ctrl_effort]
R1 = zeros(num_val, 6); % Classic PID
R2 = zeros(num_val, 6); % Gain Scheduling
R3 = zeros(num_val, 6); % GA
R4 = zeros(num_val, 6); % GS + GA

for v = 1:num_val
    vs = validation_seeds(v);
    fprintf('  Seed %d/%d (seed=%d)...\n', v, num_val, vs);
    
    % Method 1: Classic PID (delay-stable: Kp=10, Ki=0.5, Kd=2.0)
    R1(v,:) = run_headless_sim(10, 0.5, 2.0, 0, 0, vs, dt, g_acc, c_roll, r_limit);
    
    % Method 2: Gain Scheduling (same base + adaptive GS)
    R2(v,:) = run_headless_sim(10, 0.5, 2.0, alpha_p, alpha_d, vs, dt, g_acc, c_roll, r_limit);
    
    % Method 3: Pure GA (GA_Kp, GA_Ki, GA_Kd, no scheduling)
    R3(v,:) = run_headless_sim(GA_Kp, GA_Ki, GA_Kd, 0, 0, vs, dt, g_acc, c_roll, r_limit);
    
    % Method 4: GS + GA Hybrid (GA base + scheduling)
    R4(v,:) = run_headless_sim(GA_Kp, GA_Ki, GA_Kd, alpha_p, alpha_d, vs, dt, g_acc, c_roll, r_limit);
end

%% =========================================================
%  PHASE 4: AGGREGATION AND REPORTING
%  =========================================================
safe_avg = @(x) mean(x(~isnan(x)));

a1 = [safe_avg(R1(:,1)), safe_avg(R1(:,2)), safe_avg(R1(:,3)), safe_avg(R1(:,4)), safe_avg(R1(:,5)), safe_avg(R1(:,6))];
a2 = [safe_avg(R2(:,1)), safe_avg(R2(:,2)), safe_avg(R2(:,3)), safe_avg(R2(:,4)), safe_avg(R2(:,5)), safe_avg(R2(:,6))];
a3 = [safe_avg(R3(:,1)), safe_avg(R3(:,2)), safe_avg(R3(:,3)), safe_avg(R3(:,4)), safe_avg(R3(:,5)), safe_avg(R3(:,6))];
a4 = [safe_avg(R4(:,1)), safe_avg(R4(:,2)), safe_avg(R4(:,3)), safe_avg(R4(:,4)), safe_avg(R4(:,5)), safe_avg(R4(:,6))];

% Calculate Total Score (Relative to Classic PID Base = 100)
% Weights: Rise(5%), Overshoot(10%), Recovery(20%), RMS(30%), ITAE(25%), Effort(10%)
w = [0.05, 0.10, 0.20, 0.30, 0.25, 0.10];
score = @(a) 100 * sum(w .* max(0, 2 - (a ./ a1)));

s1 = score(a1);
s2 = score(a2);
s3 = score(a3);
s4 = score(a4);

base_Kp = 10.0; base_Ki = 0.5; base_Kd = 2.0;
str1 = sprintf('1. Classic PID (%.1f, %.1f, %.1f)', base_Kp, base_Ki, base_Kd);
str2 = sprintf('2. Gain Sched. (%.1f, %.1f, %.1f)', base_Kp, base_Ki, base_Kd);
str3 = sprintf('3. Genetic Alg (%.1f, %.1f, %.1f)', GA_Kp, GA_Ki, GA_Kd);
str4 = sprintf('4. Hybrid GS+GA(%.1f, %.1f, %.1f)', GA_Kp, GA_Ki, GA_Kd);

fprintf('\n============================================================================================================================================================\n');
fprintf('                                                   FINAL BENCHMARK RESULTS (Averaged over %d tests)\n', num_val);
fprintf('============================================================================================================================================================\n');
fprintf(' %-38s | Rise Time (s) | Overshoot (%%) | Recovery (s) | RMS Err (cm) | ITAE Score | Ctrl Effort (deg) | TOTAL SCORE\n', 'Method (Kp, Ki, Kd)');
fprintf('------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' %-38s | %13.3f | %13.2f | %12.3f | %12.3f | %10.2f | %17.3f | %11.1f\n', str1, a1(1), a1(2), a1(3), a1(4), a1(5), a1(6), s1);
fprintf(' %-38s | %13.3f | %13.2f | %12.3f | %12.3f | %10.2f | %17.3f | %11.1f\n', str2, a2(1), a2(2), a2(3), a2(4), a2(5), a2(6), s2);
fprintf(' %-38s | %13.3f | %13.2f | %12.3f | %12.3f | %10.2f | %17.3f | %11.1f\n', str3, a3(1), a3(2), a3(3), a3(4), a3(5), a3(6), s3);
fprintf(' %-38s | %13.3f | %13.2f | %12.3f | %12.3f | %10.2f | %17.3f | %11.1f\n', str4, a4(1), a4(2), a4(3), a4(4), a4(5), a4(6), s4);
fprintf('============================================================================================================================================================\n\n');


%% =========================================================
%  HEADLESS SIMULATION FUNCTION
%  Replicates the EXACT physics loop from stewart_pid_sim.m
%  with NO animation, NO interp1. Impulse kicks only.
%% =========================================================
function res = run_headless_sim(Kp_base, Ki_base, Kd_base, alpha_p, alpha_d, seed, dt, g_acc, c_roll, r_limit)
    % --- Build params struct for simulate_ball ---
    params.dt       = dt;
    params.T_sim    = 30;
    params.g_acc    = g_acc;
    params.c_roll   = c_roll;
    params.r_limit  = r_limit;
    params.max_tilt = 30 * (pi/180);
    params.ball_x0  = 0.05;
    params.ball_y0  = 0.03;

    % Enable Gain Scheduling if requested
    if alpha_p > 0 || alpha_d > 0
        params.gs_alpha_p = alpha_p;
        params.gs_alpha_d = alpha_d;
        params.gs_alpha_i = 10.0;
        params.gs_beta_i  = 15.0;
    end

    % --- Generate disturbances and noise ---
    disturb_table = generate_disturbances(seed, params.T_sim);
    noise_table   = generate_sensor_noise(seed, params.T_sim, dt);

    % --- Run unified physics engine ---
    result = simulate_ball(Kp_base, Ki_base, Kd_base, params, disturb_table, noise_table);

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

    % 2. Overshoot
    if isempty(disturb_table)
        t_first_dist = t_vec(end);
    else
        t_first_dist = disturb_table(1,1);
    end
    idx_first_dist = find(t_vec >= t_first_dist, 1, 'first');

    if ~isempty(idx_10) && idx_10 < idx_first_dist
        max_overshoot = max(dist_cm(idx_10:idx_first_dist));
        overshoot_pct = (max_overshoot / d0) * 100;
    else
        % System never settled before disturbance, overshoot is effectively max error
        overshoot_pct = (max(dist_cm(1:idx_first_dist)) / d0) * 100;
    end

    % 3. Recovery Time
    rec_thresh = 0.5;
    recovery_times = [];
    for d = 1:size(disturb_table,1)
        t_d = disturb_table(d,1);
        idx_d = find(t_vec >= t_d, 1, 'first');
        if d < size(disturb_table,1)
            idx_next = find(t_vec >= disturb_table(d+1,1), 1, 'first');
        else
            idx_next = length(t_vec);
        end

        window_dist = dist_cm(idx_d:idx_next);
        is_rec = window_dist < rec_thresh;
        idx_unrec = find(~is_rec, 1, 'last');

        if ~isempty(idx_unrec) && idx_unrec < length(window_dist)
            t_rec = t_vec(idx_d + idx_unrec) - t_d;
            recovery_times(end+1) = t_rec; %#ok<AGROW>
        else
            % System never recovered from this disturbance
            recovery_times(end+1) = (idx_next - idx_d) * dt; %#ok<AGROW>
        end
    end
    
    if ~isempty(recovery_times)
        avg_recovery = mean(recovery_times);
    else
        avg_recovery = params.T_sim; % Penalty if no disturbances or completely failed
    end

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

    res = [rise_time, overshoot_pct, avg_recovery, rms_err, itae, ctrl_effort];
end

