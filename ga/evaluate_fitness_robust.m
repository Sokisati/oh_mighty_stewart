function [fitness, robust_score, num_drops] = evaluate_fitness_robust(K_pid, h0, r_limit, g_acc, c_roll, max_tilt, disturbances, noises, ctrl_lambda)

    if nargin < 9 || isempty(ctrl_lambda), ctrl_lambda = 0; end

    config = load_config();

    Kp = K_pid(1);
    Ki = K_pid(2);
    Kd = K_pid(3);

    params.dt       = 0.02;
    params.T_sim    = 30.0;
    params.g_acc    = g_acc;
    params.c_roll   = c_roll;
    params.r_limit  = r_limit;
    params.max_tilt = max_tilt;
    params.ball_x0  = 0.05;
    params.ball_y0  = 0.03;

    params.gs_alpha_p = 0;
    params.gs_alpha_i = 0;
    params.gs_alpha_d = 0;

    num_scenarios = length(disturbances);
    has_drop = false;
    
    ref_metrics = [0.170, 0.28, 2.088, 1.748, 6.08, 11.752];
    w = [0.05, 0.10, 0.20, 0.30, 0.25, 0.10]; % Benchmark weights

    costs = zeros(num_scenarios, 1);
    human_scores = zeros(num_scenarios, 1);

    num_drops = 0;

    for s = 1:num_scenarios
        result = simulate_ball(Kp, Ki, Kd, params, disturbances{s}, noises{s});
        
        if result.fell_off
            num_drops = num_drops + 1;
            t_drop = result.t_vec(end);
            
            costs(s) = 2.5 + (30.0 - t_drop) * 0.05; 
            
            human_scores(s) = config.drop_penalty;
            continue;
        end
        
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

        if isempty(disturbances{s})
            t_wind_start = t_vec(end);
        else
            t_wind_start = disturbances{s}(1,1);
        end
        idx_wind_eval = find(t_vec >= (t_wind_start + 1.0), 1, 'first');
        if isempty(idx_wind_eval), idx_wind_eval = 1; end
        wind_rejection = rms(dist_cm(idx_wind_eval:end));

        idx_last_2s = find(t_vec >= t_vec(end)-2.0, 1, 'first');
        if isempty(idx_last_2s), idx_last_2s = 1; end
        rms_err = rms(dist_cm(idx_last_2s:end));

        itae = result.itae;

        ctrl_effort = rms(sqrt(result.pitch_act.^2 + result.roll_act.^2)) * (180/pi);

        metrics = [rise_time, settling_time, wind_rejection, rms_err, itae, ctrl_effort];
        
        costs(s) = sum(w .* (metrics ./ ref_metrics));
        
        human_scores(s) = 100 * sum(w .* max(0, 2 - (metrics ./ ref_metrics)));
    end

    fitness = mean(costs);
    
    robust_score = mean(human_scores);
end
