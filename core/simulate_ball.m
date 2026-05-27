function result = simulate_ball(Kp, Ki, Kd, params, disturb_table, noise_table)
% SIMULATE_BALL  Unified headless ball-on-plate physics engine.
%
%   result = simulate_ball(Kp, Ki, Kd, params, disturb_table, noise_table)
%
%   This is the SINGLE SOURCE OF TRUTH for ball dynamics and PID control.
%   All simulation modes (PID, GS, GA fitness, Benchmark) call this function.
%
%   Inputs:
%     Kp, Ki, Kd       - PID gains (base values)
%     params           - struct with fields:
%       .dt            - time step [s]
%       .T_sim         - simulation duration [s]
%       .g_acc         - gravity [m/s^2]
%       .c_roll        - rolling damping [1/s]
%       .r_limit       - plate boundary radius [m]
%       .max_tilt      - max platform tilt [rad]
%       .ball_x0       - initial ball x [m] (default 0.05)
%       .ball_y0       - initial ball y [m] (default 0.03)
%       .delay_steps   - actuator loop delay in steps (default 3 = 60ms at dt=0.02)
%                        Models: sensor read + compute + servo physical response.
%                        Set to 0 to disable delay (ideal simulation).
%       .gs_alpha_p    - Gain Scheduling alpha for Kp (0 = off)
%       .gs_alpha_i    - Gain Scheduling alpha for Ki (0 = off)
%       .gs_alpha_d    - Gain Scheduling alpha for Kd (0 = off)
%       .gs_beta_i     - Smart Dynamic Suppression beta for Ki (default 15)
%     disturb_table    - Nx3 [t_start, ax_wind, ay_wind]
%                        Continuous wind acceleration [m/s^2] active from t_start
%                        until the NEXT row's t_start. Requires Ki to eliminate
%                        the steady-state position offset this creates.
%                        Use [] for no wind.
%
%   Output:
%     result - struct with fields:
%       .ball_x, .ball_y       - ball position arrays [m]
%       .ball_vx, .ball_vy     - ball velocity arrays [m/s]
%       .pitch_cmd, .roll_cmd  - PID command arrays [rad] (what PID computed)
%       .pitch_act, .roll_act  - actual applied tilt arrays [rad] (delayed)
%       .Kp_log, .Ki_log, .Kd_log - gain log arrays
%       .t_vec                 - time vector [s]
%       .itae                  - ITAE fitness score
%       .fell_off              - true if ball left the plate
%       .fall_time             - time of fall-off [s] or NaN

    % --- Default parameters ---
    dt       = params.dt;
    T_sim    = params.T_sim;
    g_acc    = params.g_acc;
    c_roll   = params.c_roll;
    r_limit  = params.r_limit;
    max_tilt = params.max_tilt;

    if isfield(params, 'ball_x0'), ball_x0 = params.ball_x0; else, ball_x0 = 0.05; end
    if isfield(params, 'ball_y0'), ball_y0 = params.ball_y0; else, ball_y0 = 0.03; end

    % Load default delay and slew rate from config.txt as a struct
    config = load_config();

    % Actuator delay in seconds (from config.txt or overridden by params)
    if isfield(params, 'delay_sec')
        delay_sec = params.delay_sec;
    else
        delay_sec = config.delay_sec;
    end
    
    % Calculate fractional delay steps based on dt
    delay_steps = max(0, delay_sec / dt);
    int_delay   = floor(delay_steps);
    frac_delay  = delay_steps - int_delay;
    
    % Slew rate (from config.txt or overridden by params)
    if isfield(params, 'slew_rate')
        slew_rate = params.slew_rate;
    else
        slew_rate = config.slew_rate_deg * (pi/180);
    end

    % Gain Scheduling parameters removed (all GS logic deleted)

    % Handle empty disturbance/noise inputs
    if nargin < 5 || isempty(disturb_table), disturb_table = zeros(0, 3); end
    has_noise = (nargin >= 6 && ~isempty(noise_table));

    % --- State arrays ---
    N = round(T_sim / dt);
    t_vec = linspace(0, T_sim, N)';

    ball_x  = zeros(N, 1);
    ball_y  = zeros(N, 1);
    ball_vx = zeros(N, 1);
    ball_vy = zeros(N, 1);
    pitch_log = zeros(N, 1);  % what PID computed (command)
    roll_log  = zeros(N, 1);
    pitch_act = zeros(N, 1);  % what was actually applied (delayed)
    roll_act  = zeros(N, 1);
    Kp_log = zeros(N, 1);
    Ki_log = zeros(N, 1);
    Kd_log = zeros(N, 1);

    ball_x(1) = ball_x0;
    ball_y(1) = ball_y0;

    % --- Actuator delay circular buffer ---
    % Stores the last few commands for fractional interpolation.
    buf_size  = int_delay + 2; % +1 for current, +1 for fractional interpolation
    pitch_buf = zeros(buf_size, 1);
    roll_buf  = zeros(buf_size, 1);
    buf_idx   = 1;  % next write position (1-indexed, wraps around)

    % Physical state of the platform (for Slew Rate Limiting)
    pitch_phys = 0;
    roll_phys = 0;

    % PID state
    int_ex = 0;  int_ey = 0;
    if has_noise
        prev_ex = -(ball_x(1) + noise_table(1, 2));
        prev_ey =   ball_y(1) + noise_table(1, 3);
    else
        prev_ex = -ball_x(1);
        prev_ey =  ball_y(1);
    end

    % LPF state for Smart Dynamic Suppression
    err_dot_filtered = 0;

    % Fitness accumulators
    itae = 0;
    fell_off = false;
    fall_time = NaN;

    % --- Fixed PID Gains ---
    Kp_eff = Kp;
    Ki_eff = Ki;
    Kd_eff = Kd;

    % --- Discrete disturbances (velocity kicks) ---
    has_kick = (size(disturb_table, 1) > 0);
    next_kick = 1;

    % --- Physics loop ---
    for i = 1:N-1
        % 1. Apply discrete velocity kicks
        if has_kick && next_kick <= size(disturb_table, 1) && t_vec(i) >= disturb_table(next_kick, 1)
            ball_vx(i) = ball_vx(i) + disturb_table(next_kick, 2);
            ball_vy(i) = ball_vy(i) + disturb_table(next_kick, 3);
            next_kick = next_kick + 1;
        end

        % 2. Sensor reading (with or without noise)
        if has_noise
            measured_x = ball_x(i) + noise_table(i, 2);
            measured_y = ball_y(i) + noise_table(i, 3);
        else
            measured_x = ball_x(i);
            measured_y = ball_y(i);
        end

        % 3. Error signals
        ex = -measured_x;
        ey =  measured_y;

        % 4. Derivative
        dex = (ex - prev_ex) / dt;
        dey = (ey - prev_ey) / dt;
        prev_ex = ex;
        prev_ey = ey;

        % 5. Adaptive PID Updates (MRAC)
        if isfield(params, 'mrac') && params.mrac.active
            % True Gradient Descent MRAC Laws
            e_pos_sq = ex^2 + ey^2;
            corr_i   = ex * int_ex + ey * int_ey;
            corr_d   = ex * dex + ey * dey;
            
            % Advanced Robust Dual Dead-Zone:
            err_mag = sqrt(e_pos_sq);
            
            % 1. PD Deadzone (2.0 cm): Kp and Kd only act as an emergency safety net 
            % for large deviations. If error < 2cm, they stay at optimal baseline.
            if err_mag > 0.020
                adapt_scale_pd = (err_mag - 0.020) / err_mag; 
            else
                adapt_scale_pd = 0.0;
            end
            
            % 2. I Deadzone (0.5 cm): Ki activates early to fight steady wind, 
            % and FREEZES inside the deadzone to hold the ball perfectly.
            if err_mag > 0.005
                adapt_scale_i = (err_mag - 0.005) / err_mag;
            else
                adapt_scale_i = 0.0;
            end
            
            % Fast elastic recovery for Kp and Kd (snaps back to baseline quickly)
            sigma_pd = 5.0; 
            
            % Smooth Adaptation
            dKp = params.mrac.gamma_p * (e_pos_sq * adapt_scale_pd) - sigma_pd * (Kp_eff - Kp);
            dKd = params.mrac.gamma_d * (corr_d * adapt_scale_pd)   - sigma_pd * (Kd_eff - Kd);
            
            % Ki: Leakage is scaled by adapt_scale_i (Freezes inside deadzone)
            dKi = params.mrac.gamma_i * (corr_i * adapt_scale_i) - params.mrac.sigma_i * adapt_scale_i * (Ki_eff - Ki);
            
            % BALANCED LIMITS for Robust MRAC:
            Kp_eff = max(Kp, min(Kp * 1.15, Kp_eff + dKp * dt));
            Ki_eff = max(Ki, min(Ki * 5.0, Ki_eff + dKi * dt));
            Kd_eff = max(Kd, min(Kd * 2.0, Kd_eff + dKd * dt));
        elseif isfield(params, 'nlpid') && params.nlpid.active
            % Non-Linear Expert PID (NLPID) - Robust Version
            err_mag = sqrt(ex^2 + ey^2);
            int_mag = sqrt(int_ex^2 + int_ey^2);
            
            % 1. Danger Reflex (Quadratic scaling ignores small noise, reacts strongly to large error)
            % If err_mag = e_scale, ratio is 1.0. If err_mag is half, ratio is 0.25.
            e_ratio = min(1.0, (err_mag / params.nlpid.e_scale)^2);
            
            % 2. Target Gains based on Expert Rules
            % Boost Kp and Kd together to act as an elastic wall without losing damping
            Kp_target = Kp + params.nlpid.kp_boost * e_ratio;
            Kd_target = Kd + params.nlpid.kd_boost * e_ratio;
            
            % 3. Wind Rejection (Integral Boost)
            % If integral is building up (steady wind), boost Ki quadratically
            i_ratio = min(1.0, (int_mag / params.nlpid.i_scale)^2);
            Ki_target = Ki + params.nlpid.ki_boost * i_ratio;
            
            % 4. Smooth Application (Low-Pass Filter)
            % Expert logic computes targets instantly, but physical motors need smooth transitions.
            % We use a 50ms time constant to prevent violent jitter.
            tau = 0.05; 
            alpha_filter = dt / (tau + dt);
            
            Kp_eff = Kp_eff + alpha_filter * (Kp_target - Kp_eff);
            Ki_eff = Ki_eff + alpha_filter * (Ki_target - Ki_eff);
            Kd_eff = Kd_eff + alpha_filter * (Kd_target - Kd_eff);
        end
        
        Kp_log(i) = Kp_eff;
        Ki_log(i) = Ki_eff;
        Kd_log(i) = Kd_eff;


        % 6. Integral with Scale-Aware Anti-Windup
        int_ex = int_ex + ex * dt;
        int_ey = int_ey + ey * dt;
        
        % Scale-Aware Anti-Windup: Integral should not command more than 30% of max_tilt
        int_limit = (max_tilt * 0.3) / max(0.01, Ki_eff);
        int_ex = max(-int_limit, min(int_limit, int_ex));
        int_ey = max(-int_limit, min(int_limit, int_ey));

        % 7. PID output (this is the COMMANDED value at time i)
        pitch_cmd_i = Kp_eff*ex + Ki_eff*int_ex + Kd_eff*dex;
        roll_cmd_i  = Kp_eff*ey + Ki_eff*int_ey + Kd_eff*dey;

        pitch_cmd_i = max(-max_tilt, min(max_tilt, pitch_cmd_i));
        roll_cmd_i  = max(-max_tilt, min(max_tilt, roll_cmd_i));

        pitch_log(i) = pitch_cmd_i;
        roll_log(i)  = roll_cmd_i;

        % 8. Actuator delay & Slew Rate Limiting
        % Buffer insertion
        pitch_buf(buf_idx) = pitch_cmd_i;
        roll_buf(buf_idx)  = roll_cmd_i;
        
        % Read from delay buffer with fractional interpolation
        idx_1 = mod(buf_idx - int_delay - 1, buf_size) + 1;
        idx_2 = mod(buf_idx - int_delay - 2, buf_size) + 1;
        
        pitch_target = (1 - frac_delay) * pitch_buf(idx_1) + frac_delay * pitch_buf(idx_2);
        roll_target  = (1 - frac_delay) * roll_buf(idx_1)  + frac_delay * roll_buf(idx_2);
        
        buf_idx = mod(buf_idx, buf_size) + 1;

        % Slew Rate Limiting (Motor velocity limit)
        max_step = slew_rate * dt;
        
        pitch_diff = pitch_target - pitch_phys;
        pitch_step = max(-max_step, min(max_step, pitch_diff));
        pitch_phys = pitch_phys + pitch_step;
        
        roll_diff = roll_target - roll_phys;
        roll_step = max(-max_step, min(max_step, roll_diff));
        roll_phys = roll_phys + roll_step;

        pitch_applied = pitch_phys;
        roll_applied  = roll_phys;

        pitch_act(i) = pitch_applied;
        roll_act(i)  = roll_applied;

        % 9. ITAE fitness + fall-off check (based on real ball position)
        dist_from_center = sqrt(ball_x(i)^2 + ball_y(i)^2);
        if dist_from_center > r_limit
            itae = itae + 10000 * (N - i);  % death penalty
            fell_off = true;
            fall_time = t_vec(i);
            Kp_log(i:end) = Kp_eff;
            Ki_log(i:end) = Ki_eff;
            Kd_log(i:end) = Kd_eff;
            pitch_log(i:end) = pitch_cmd_i;
            roll_log(i:end) = roll_cmd_i;
            break;
        end
        t_i = i * dt;
        itae = itae + (t_i * dist_from_center * dt);

        % 10. Ball dynamics with DELAYED tilt (Symplectic Euler)
        ax = (5/7) * g_acc * sin(pitch_applied) - c_roll * ball_vx(i);
        ay = -(5/7) * g_acc * sin(roll_applied)  - c_roll * ball_vy(i);

        ball_vx(i+1) = ball_vx(i) + ax * dt;
        ball_vy(i+1) = ball_vy(i) + ay * dt;
        ball_x(i+1)  = ball_x(i)  + ball_vx(i+1) * dt;
        ball_y(i+1)  = ball_y(i)  + ball_vy(i+1) * dt;
    end
    
    % Log final step values (N) to prevent trailing zeros due to vector pre-allocation
    if ~fell_off
        Kp_log(N) = Kp_eff;
        Ki_log(N) = Ki_eff;
        Kd_log(N) = Kd_eff;
        pitch_log(N) = pitch_log(N-1);
        roll_log(N)  = roll_log(N-1);
        pitch_act(N) = pitch_act(N-1);
        roll_act(N)  = roll_act(N-1);
    end

    % --- Pack results ---
    result.ball_x    = ball_x;
    result.ball_y    = ball_y;
    result.ball_vx   = ball_vx;
    result.ball_vy   = ball_vy;
    result.pitch_cmd = pitch_log;   % commanded (before delay)
    result.roll_cmd  = roll_log;
    result.pitch_act = pitch_act;   % actually applied (after delay)
    result.roll_act  = roll_act;
    result.Kp_log    = Kp_log;
    result.Ki_log    = Ki_log;
    result.Kd_log    = Kd_log;
    result.t_vec     = t_vec;
    result.itae      = itae;
    result.fell_off  = fell_off;
    result.fall_time = fall_time;
    result.N         = N;
    result.delay_steps = delay_steps;
end
