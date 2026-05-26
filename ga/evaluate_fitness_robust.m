function fitness = evaluate_fitness_robust(K_pid, h0, r_limit, g_acc, c_roll, max_tilt, disturbances, noises, ctrl_lambda)
% EVALUATE_FITNESS_ROBUST  Multi-objective fitness across multiple wind/noise scenarios.
%   fitness = avg_ITAE + ctrl_lambda * avg_control_effort
%
%   K_pid       = [Kp, Ki, Kd]
%   ctrl_lambda = penalty weight for control effort (default 0 → pure ITAE)
%                 Recommended: 0.03  (keeps aggressive Kd in check without
%                 dominating the ITAE term)

    if nargin < 9 || isempty(ctrl_lambda), ctrl_lambda = 0; end

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

    % No Gain Scheduling (plain PID)
    params.gs_alpha_p = 0;
    params.gs_alpha_i = 0;
    params.gs_alpha_d = 0;

    total_itae   = 0;
    total_effort = 0;
    num_scenarios = length(disturbances);

    for s = 1:num_scenarios
        result = simulate_ball(Kp, Ki, Kd, params, disturbances{s}, noises{s});
        total_itae = total_itae + result.itae;

        if ctrl_lambda > 0
            % RMS of actual applied tilt magnitude (degrees) — penalises excessive actuation
            effort = rms(sqrt(result.pitch_act.^2 + result.roll_act.^2)) * (180/pi);
            total_effort = total_effort + effort;
        end
    end

    avg_itae   = total_itae   / num_scenarios;
    avg_effort = total_effort / num_scenarios;

    fitness = avg_itae + ctrl_lambda * avg_effort;
end
