function fitness = evaluate_fitness_robust(K_pid, h0, r_limit, g_acc, c_roll, max_tilt, disturbances, noises)
% EVALUATE_FITNESS_ROBUST Evaluates PID under multiple wind/noise scenarios
% Calculates the average ITAE score across all scenarios. Lower is better.
%
% K_pid = [Kp, Ki, Kd]

    Kp = K_pid(1);
    Ki = K_pid(2);
    Kd = K_pid(3);

    % Build params struct for simulate_ball
    params.dt       = 0.02;
    params.T_sim    = 30.0;
    params.g_acc    = g_acc;
    params.c_roll   = c_roll;
    params.r_limit  = r_limit;
    params.max_tilt = max_tilt;
    params.ball_x0  = 0.05;
    params.ball_y0  = 0.03;

    % No Gain Scheduling (plain PID)
    params.gs_alpha_p = 0;
    params.gs_alpha_i = 0;
    params.gs_alpha_d = 0;

    total_fitness = 0;
    num_scenarios = length(disturbances);

    for s = 1:num_scenarios
        result = simulate_ball(Kp, Ki, Kd, params, disturbances{s}, noises{s});
        total_fitness = total_fitness + result.itae;
    end

    % Return average fitness across all scenarios
    fitness = total_fitness / num_scenarios;
end
