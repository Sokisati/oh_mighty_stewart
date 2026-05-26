function fitness = evaluate_fitness(K_pid, h0, r_limit, g_acc, c_roll, max_tilt)
% EVALUATE_FITNESS Headless, fast simulation for Genetic Algorithm tuning
% Calculates the ITAE (Integral Time Absolute Error) score. Lower is better.
%
% K_pid = [Kp, Ki, Kd]

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

    % Run simulation (no disturbances, no noise)
    result = simulate_ball(K_pid(1), K_pid(2), K_pid(3), params);

    fitness = result.itae;
end
