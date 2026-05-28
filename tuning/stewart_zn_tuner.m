
clc; close all;

this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(this_dir, '..', 'core')));
addpath(genpath(fullfile(this_dir, '..', 'environment')));

fprintf('========================================================\n');
fprintf('       ZIEGLER-NICHOLS AUTONOMOUS TUNING MODULE\n');
fprintf('========================================================\n\n');

conf = load_config();

params.dt       = conf.dt;
params.T_sim    = 20.0; % Daha uzun simülasyon, salınımları net görmek için
params.g_acc    = conf.g_acc;
params.c_roll   = conf.c_roll;
params.r_limit  = conf.R_base; 
params.max_tilt = conf.max_tilt_deg * (pi / 180);
params.delay_sec = conf.delay_sec;
params.slew_rate = conf.slew_rate_deg * (pi / 180);

params.ball_x0 = 0.05; 
params.ball_y0 = 0.0;

disturb_table = [];
noise_table   = [];

Kp_low = 0.1;
Kp_high = 25.0; % Daha hızlı motorlar için sınır yükseltildi
max_iters = 25;

fprintf('Searching for Ku (Ultimate Gain)...\n\n');

Ku = 0;
Tu = 0;
best_res = []; % En iyi (sabit) salınım sonucunu kaydetmek için
best_idx_peaks = [];

for iter = 1:max_iters
    Kp_test = (Kp_low + Kp_high) / 2;
    
    Ki_test = 0;
    Kd_test = 0;
    
    res = simulate_ball(Kp_test, Ki_test, Kd_test, params, disturb_table, noise_table);
    
    if res.fell_off
        fprintf('Iter %2d | Kp = %6.3f | RESULT: Fell off (Unstable) -> Decreasing Kp\n', iter, Kp_test);
        Kp_high = Kp_test;
        continue;
    end
    
    r_pos = sqrt(res.ball_x.^2 + res.ball_y.^2);
    
    idx_peaks = find(r_pos(2:end-1) > r_pos(1:end-2) & r_pos(2:end-1) > r_pos(3:end)) + 1;
    
    t_peaks = res.t_vec(idx_peaks);
    valid_mask = t_peaks > 2.0;
    idx_peaks = idx_peaks(valid_mask);
    t_peaks = t_peaks(valid_mask);
    
    if length(idx_peaks) < 5
        fprintf('Iter %2d | Kp = %6.3f | RESULT: Overdamped (No oscillation) -> Increasing Kp\n', iter, Kp_test);
        Kp_low = Kp_test;
        continue;
    end
    
    peaks_amp = r_pos(idx_peaks(end-3:end));
    t_local = (1:4)';
    slope = (t_local' * peaks_amp - mean(t_local) * sum(peaks_amp)) / (t_local' * t_local - 4 * mean(t_local)^2);
    norm_slope = slope / mean(peaks_amp); % normalize edilmiş eğim
    
    Tu_est = mean(diff(t_peaks(end-3:end)));
    
    tol = 0.015;
    if norm_slope > tol
        fprintf('Iter %2d | Kp = %6.3f | RESULT: Diverging Oscillation (slope=%.4f) -> Decreasing Kp\n', iter, Kp_test, norm_slope);
        Kp_high = Kp_test;
        Ku = Kp_test;
        Tu = Tu_est;
        best_res = res;
        best_idx_peaks = idx_peaks;
    elseif norm_slope < -tol
        fprintf('Iter %2d | Kp = %6.3f | RESULT: Damping Out (slope=%.4f) -> Increasing Kp\n', iter, Kp_test, norm_slope);
        Kp_low = Kp_test;
        if Ku == 0
            best_res = res;
            best_idx_peaks = idx_peaks;
        end
    else
        fprintf('Iter %2d | Kp = %6.3f | RESULT: MARGINALLY STABLE OSCILLATION FOUND! (slope=%.4f)\n', iter, Kp_test, norm_slope);
        Ku = Kp_test;
        Tu = Tu_est;
        best_res = res;
        best_idx_peaks = idx_peaks;
        break;
    end
    
    if iter == max_iters
        fprintf('\n[WARNING] Maximum iterations reached. Using the closest stable value.\n');
        if isempty(best_res)
            best_res = res;
            best_idx_peaks = idx_peaks;
        end
    end
end

if ~isempty(best_res)
    r_plot = sqrt(best_res.ball_x.^2 + best_res.ball_y.^2) * 100; % cm
    fig = figure('Name', 'Ziegler-Nichols Stability Margin', 'Position', [100, 100, 900, 500], 'Color', 'w');
    
    sgtitle(sprintf('Ziegler-Nichols Method: Constant Amplitude Oscillation Analysis\nUltimate Gain (Ku) = %.3f, Ultimate Period (Tu) = %.3f s', Ku, Tu), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

    ax1 = subplot(2,1,1);
    plot(ax1, best_res.t_vec, best_res.ball_x * 100, '-', 'Color', [0 0.447 0.741], 'LineWidth', 2.0); hold(ax1, 'on');
    plot(ax1, best_res.t_vec, best_res.ball_y * 100, '-', 'Color', [0.850 0.325 0.098], 'LineWidth', 2.0);
    yline(ax1, 0, 'k--', 'LineWidth', 1.0);
    title(ax1, 'Axial Position Tracking (X and Y)', 'FontSize', 12, 'Color', 'k');
    xlabel(ax1, 'Time (s)', 'FontSize', 11, 'Color', 'k'); 
    ylabel(ax1, 'Position (cm)', 'FontSize', 11, 'Color', 'k');
    legend(ax1, 'X Axis', 'Y Axis', 'Center (0 cm)', 'Location', 'northeast', 'Color', 'w', 'TextColor', 'k', 'EdgeColor', [0.8 0.8 0.8]); 
    grid(ax1, 'on');
    set(ax1, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.15, 'FontSize', 10);
    
    ax2 = subplot(2,1,2);
    plot(ax2, best_res.t_vec, r_plot, '-', 'Color', [0.466 0.674 0.188], 'LineWidth', 2.0); hold(ax2, 'on');
    plot(ax2, best_res.t_vec(best_idx_peaks), r_plot(best_idx_peaks), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    
    mean_amp = mean(r_plot(best_idx_peaks));
    yline(ax2, mean_amp, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Mean Oscillation Amplitude');
    
    title(ax2, 'Radial Oscillation Amplitude and Ultimate Period (Tu) Detection', 'FontSize', 12, 'Color', 'k');
    xlabel(ax2, 'Time (s)', 'FontSize', 11, 'Color', 'k'); 
    ylabel(ax2, 'Radial Distance (cm)', 'FontSize', 11, 'Color', 'k');
    legend(ax2, 'System Response', 'Peaks', 'Mean Amplitude', 'Location', 'northeast', 'Color', 'w', 'TextColor', 'k', 'EdgeColor', [0.8 0.8 0.8]); 
    grid(ax2, 'on');
    set(ax2, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.15, 'FontSize', 10);
    
    dim = [0.15 0.01 0.7 0.06];
    annotation('textbox', dim, 'String', sprintf('Note: At Kp = %.3f, marginally stable (constant amplitude) oscillations were achieved without damping or diverging.', Ku), 'FitBoxToText', 'on', 'BackgroundColor', [0.95 0.95 0.95], 'EdgeColor', 'k', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');

    drawnow;
    
    fprintf('\n[+] Visualizing marginally stable oscillations at Kp = %.3f in 3D...\n', Ku);
    visualize_stewart(best_res, Ku, 0, 0, []);
end

fprintf('\n========================================================\n');
fprintf('  PHYSICAL SYSTEM LIMITS (Ku and Tu)\n');
fprintf('--------------------------------------------------------\n');
fprintf(' Ultimate Gain (Ku)   : %.3f\n', Ku);
fprintf(' Ultimate Period (Tu) : %.3f seconds\n', Tu);
fprintf('========================================================\n\n');

zn_Kp = 0.6 * Ku;
zn_Ki = 1.2 * Ku / Tu;
zn_Kd = 0.075 * Ku * Tu;

pessen_Kp = 0.7 * Ku;
pessen_Ki = 1.75 * Ku / Tu;
pessen_Kd = 0.105 * Ku * Tu;

so_Kp = 0.33 * Ku;
so_Ki = 0.66 * Ku / Tu;
so_Kd = 0.11 * Ku * Tu;

no_Kp = 0.2 * Ku;
no_Ki = 0.4 * Ku / Tu;
no_Kd = 0.066 * Ku * Tu;

fprintf('========================================================\n');
fprintf('  RECOMMENDED ZIEGLER-NICHOLS PARAMETERS\n');
fprintf('--------------------------------------------------------\n');
fprintf(' [1] Classic Z-N (Aggressive):\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n\n', zn_Kp, zn_Ki, zn_Kd);
fprintf(' [2] Pessen Integral Rule (Fast Response):\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n\n', pessen_Kp, pessen_Ki, pessen_Kd);
fprintf(' [3] Some Overshoot:\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n\n', so_Kp, so_Ki, so_Kd);
fprintf(' [4] No Overshoot:\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n', no_Kp, no_Ki, no_Kd);
fprintf('========================================================\n');
