function result = simulate_ball(Kp, Ki, Kd, params, disturb_table, noise_table)

    dt       = params.dt;
    T_sim    = params.T_sim;
    g_acc    = params.g_acc;
    c_roll   = params.c_roll;
    r_limit  = params.r_limit;
    max_tilt = params.max_tilt;

    if isfield(params, 'ball_x0'), ball_x0 = params.ball_x0; else, ball_x0 = 0.05; end
    if isfield(params, 'ball_y0'), ball_y0 = params.ball_y0; else, ball_y0 = 0.03; end

    config = load_config();

    if isfield(params, 'delay_sec')
        delay_sec = params.delay_sec;
    else
        delay_sec = config.delay_sec;
    end
    
    delay_steps = max(0, delay_sec / dt);
    int_delay   = floor(delay_steps);
    frac_delay  = delay_steps - int_delay;
    
    if isfield(params, 'slew_rate')
        slew_rate = params.slew_rate;
    else
        slew_rate = config.slew_rate_deg * (pi/180);
    end


    if nargin < 5 || isempty(disturb_table), disturb_table = zeros(0, 3); end
    has_noise = (nargin >= 6 && ~isempty(noise_table));

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

    buf_size  = int_delay + 2; % +1 for current, +1 for fractional interpolation
    pitch_buf = zeros(buf_size, 1);
    roll_buf  = zeros(buf_size, 1);
    buf_idx   = 1;  % next write position (1-indexed, wraps around)

    pitch_phys = 0;
    roll_phys = 0;

    int_ex = 0;  int_ey = 0;
    if has_noise
        prev_ex = -(ball_x(1) + noise_table(1, 2));
        prev_ey =   ball_y(1) + noise_table(1, 3);
    else
        prev_ex = -ball_x(1);
        prev_ey =  ball_y(1);
    end

    err_dot_filtered = 0;

    itae = 0;
    fell_off = false;
    fall_time = NaN;

    Kp_eff = Kp;
    Ki_eff = Ki;
    Kd_eff = Kd;

    has_kick = (size(disturb_table, 1) > 0);
    next_kick = 1;

    for i = 1:N-1
        if has_kick && next_kick <= size(disturb_table, 1) && t_vec(i) >= disturb_table(next_kick, 1)
            ball_vx(i) = ball_vx(i) + disturb_table(next_kick, 2);
            ball_vy(i) = ball_vy(i) + disturb_table(next_kick, 3);
            next_kick = next_kick + 1;
        end

        if has_noise
            measured_x = ball_x(i) + noise_table(i, 2);
            measured_y = ball_y(i) + noise_table(i, 3);
        else
            measured_x = ball_x(i);
            measured_y = ball_y(i);
        end

        ex = -measured_x;
        ey =  measured_y;

        dex = (ex - prev_ex) / dt;
        dey = (ey - prev_ey) / dt;
        prev_ex = ex;
        prev_ey = ey;

        if isfield(params, 'mrac') && params.mrac.active
            e_pos_sq = ex^2 + ey^2;
            corr_i   = ex * int_ex + ey * int_ey;
            corr_d   = ex * dex + ey * dey;
            
            err_mag = sqrt(e_pos_sq);
            
            if err_mag > 0.020
                adapt_scale_pd = (err_mag - 0.020) / err_mag; 
            else
                adapt_scale_pd = 0.0;
            end
            
            if err_mag > 0.005
                adapt_scale_i = (err_mag - 0.005) / err_mag;
            else
                adapt_scale_i = 0.0;
            end
            
            sigma_pd = 5.0; 
            
            dKp = params.mrac.gamma_p * (e_pos_sq * adapt_scale_pd) - sigma_pd * (Kp_eff - Kp);
            dKd = params.mrac.gamma_d * (corr_d * adapt_scale_pd)   - sigma_pd * (Kd_eff - Kd);
            
            dKi = params.mrac.gamma_i * (corr_i * adapt_scale_i) - params.mrac.sigma_i * adapt_scale_i * (Ki_eff - Ki);
            
            Kp_eff = max(Kp, min(Kp * 1.15, Kp_eff + dKp * dt));
            Ki_eff = max(Ki, min(Ki * 5.0, Ki_eff + dKi * dt));
            Kd_eff = max(Kd, min(Kd * 2.0, Kd_eff + dKd * dt));
        elseif isfield(params, 'nlpid') && params.nlpid.active
            err_mag = sqrt(ex^2 + ey^2);
            int_mag = sqrt(int_ex^2 + int_ey^2);
            
            e_ratio = min(1.0, (err_mag / params.nlpid.e_scale)^2);
            
            Kp_target = Kp + params.nlpid.kp_boost * e_ratio;
            Kd_target = Kd + params.nlpid.kd_boost * e_ratio;
            
            i_ratio = min(1.0, (int_mag / params.nlpid.i_scale)^2);
            Ki_target = Ki + params.nlpid.ki_boost * i_ratio;
            
            tau = 0.05; 
            alpha_filter = dt / (tau + dt);
            
            Kp_eff = Kp_eff + alpha_filter * (Kp_target - Kp_eff);
            Ki_eff = Ki_eff + alpha_filter * (Ki_target - Ki_eff);
            Kd_eff = Kd_eff + alpha_filter * (Kd_target - Kd_eff);
        end
        
        Kp_log(i) = Kp_eff;
        Ki_log(i) = Ki_eff;
        Kd_log(i) = Kd_eff;


        int_ex = int_ex + ex * dt;
        int_ey = int_ey + ey * dt;
        
        int_limit = (max_tilt * 0.3) / max(0.01, Ki_eff);
        int_ex = max(-int_limit, min(int_limit, int_ex));
        int_ey = max(-int_limit, min(int_limit, int_ey));

        pitch_cmd_i = Kp_eff*ex + Ki_eff*int_ex + Kd_eff*dex;
        roll_cmd_i  = Kp_eff*ey + Ki_eff*int_ey + Kd_eff*dey;

        pitch_cmd_i = max(-max_tilt, min(max_tilt, pitch_cmd_i));
        roll_cmd_i  = max(-max_tilt, min(max_tilt, roll_cmd_i));

        pitch_log(i) = pitch_cmd_i;
        roll_log(i)  = roll_cmd_i;

        pitch_buf(buf_idx) = pitch_cmd_i;
        roll_buf(buf_idx)  = roll_cmd_i;
        
        idx_1 = mod(buf_idx - int_delay - 1, buf_size) + 1;
        idx_2 = mod(buf_idx - int_delay - 2, buf_size) + 1;
        
        pitch_target = (1 - frac_delay) * pitch_buf(idx_1) + frac_delay * pitch_buf(idx_2);
        roll_target  = (1 - frac_delay) * roll_buf(idx_1)  + frac_delay * roll_buf(idx_2);
        
        buf_idx = mod(buf_idx, buf_size) + 1;

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

        ax = (5/7) * g_acc * sin(pitch_applied) - c_roll * ball_vx(i);
        ay = -(5/7) * g_acc * sin(roll_applied)  - c_roll * ball_vy(i);

        ball_vx(i+1) = ball_vx(i) + ax * dt;
        ball_vy(i+1) = ball_vy(i) + ay * dt;
        ball_x(i+1)  = ball_x(i)  + ball_vx(i+1) * dt;
        ball_y(i+1)  = ball_y(i)  + ball_vy(i+1) * dt;
    end
    
    if ~fell_off
        Kp_log(N) = Kp_eff;
        Ki_log(N) = Ki_eff;
        Kd_log(N) = Kd_eff;
        pitch_log(N) = pitch_log(N-1);
        roll_log(N)  = roll_log(N-1);
        pitch_act(N) = pitch_act(N-1);
        roll_act(N)  = roll_act(N-1);
    end

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
