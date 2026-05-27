
clc; close all;

this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(this_dir, '..', 'core')));
addpath(genpath(fullfile(this_dir, '..', 'environment')));

fprintf('========================================================\n');
fprintf('       ZIEGLER-NICHOLS OTONOM TUNING MODÜLÜ\n');
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

fprintf('Ku (Ultimate Gain) aranıyor...\n\n');

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
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Düştü (Kararsız) -> Kp azaltılıyor\n', iter, Kp_test);
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
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Aşırı Sönümlü (Salınım yok) -> Kp artırılıyor\n', iter, Kp_test);
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
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Salınım Büyüyor (slope=%.4f) -> Kp azaltılıyor\n', iter, Kp_test, norm_slope);
        Kp_high = Kp_test;
        Ku = Kp_test;
        Tu = Tu_est;
        best_res = res;
        best_idx_peaks = idx_peaks;
    elseif norm_slope < -tol
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Sönümleniyor (slope=%.4f) -> Kp artırılıyor\n', iter, Kp_test, norm_slope);
        Kp_low = Kp_test;
        if Ku == 0
            best_res = res;
            best_idx_peaks = idx_peaks;
        end
    else
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: SABİT SALINIM BULUNDU! (slope=%.4f)\n', iter, Kp_test, norm_slope);
        Ku = Kp_test;
        Tu = Tu_est;
        best_res = res;
        best_idx_peaks = idx_peaks;
        break;
    end
    
    if iter == max_iters
        fprintf('\n[UYARI] Maksimum iterasyona ulaşıldı. En yakın değer kullanılıyor.\n');
        if isempty(best_res)
            best_res = res;
            best_idx_peaks = idx_peaks;
        end
    end
end

if ~isempty(best_res)
    r_plot = sqrt(best_res.ball_x.^2 + best_res.ball_y.^2) * 100; % cm
    fig = figure('Name', 'Ziegler-Nichols Kararlılık Sınırı', 'Position', [100, 100, 900, 500], 'Color', 'w');
    
    sgtitle(sprintf('Ziegler-Nichols Yöntemi: Sabit Genlikli Salınım Analizi\nBulunan Kritik Kazanç (Ku) = %.3f, Kritik Periyot (Tu) = %.3f s', Ku, Tu), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

    ax1 = subplot(2,1,1);
    plot(ax1, best_res.t_vec, best_res.ball_x * 100, '-', 'Color', [0 0.447 0.741], 'LineWidth', 2.0); hold(ax1, 'on');
    plot(ax1, best_res.t_vec, best_res.ball_y * 100, '-', 'Color', [0.850 0.325 0.098], 'LineWidth', 2.0);
    yline(ax1, 0, 'k--', 'LineWidth', 1.0);
    title(ax1, 'Eksenel Konum Değişimleri (X ve Y)', 'FontSize', 12, 'Color', 'k');
    xlabel(ax1, 'Zaman (s)', 'FontSize', 11, 'Color', 'k'); 
    ylabel(ax1, 'Pozisyon (cm)', 'FontSize', 11, 'Color', 'k');
    legend(ax1, 'X Ekseni', 'Y Ekseni', 'Merkez (0 cm)', 'Location', 'northeast', 'TextColor', 'k'); 
    grid(ax1, 'on');
    set(ax1, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.15, 'FontSize', 10);
    
    ax2 = subplot(2,1,2);
    plot(ax2, best_res.t_vec, r_plot, '-', 'Color', [0.466 0.674 0.188], 'LineWidth', 2.0); hold(ax2, 'on');
    plot(ax2, best_res.t_vec(best_idx_peaks), r_plot(best_idx_peaks), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    
    mean_amp = mean(r_plot(best_idx_peaks));
    yline(ax2, mean_amp, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Ortalama Salınım Genliği');
    
    title(ax2, 'Radyal Salınım Genliği ve Kritik Periyot (Tu) Tespiti', 'FontSize', 12, 'Color', 'k');
    xlabel(ax2, 'Zaman (s)', 'FontSize', 11, 'Color', 'k'); 
    ylabel(ax2, 'Radyal Uzaklık (cm)', 'FontSize', 11, 'Color', 'k');
    legend(ax2, 'Sistem Yanıtı', 'Tepe Noktaları', 'Ortalama Genlik', 'Location', 'northeast', 'TextColor', 'k'); 
    grid(ax2, 'on');
    set(ax2, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.15, 'FontSize', 10);
    
    dim = [0.15 0.01 0.7 0.06];
    annotation('textbox', dim, 'String', sprintf('Not: Kp = %.3f kazanç değerinde, sönümlenmeyen veya büyümeyen (marjinal kararlı) sabit genlikli salınımlar elde edilmiştir.', Ku), 'FitBoxToText', 'on', 'BackgroundColor', [0.95 0.95 0.95], 'EdgeColor', 'k', 'Color', 'k', 'FontSize', 10, 'HorizontalAlignment', 'center');

    drawnow;
    
    fprintf('\n[+] Kp = %.3f değerindeki sabit salınımlar 3D olarak görselleştiriliyor...\n', Ku);
    visualize_stewart(best_res, Ku, 0, 0, []);
end

fprintf('\n========================================================\n');
fprintf('  SİSTEMİN FİZİKSEL LİMİTLERİ (Ku ve Tu)\n');
fprintf('--------------------------------------------------------\n');
fprintf(' Ultimate Gain (Ku)   : %.3f\n', Ku);
fprintf(' Ultimate Period (Tu) : %.3f saniye\n', Tu);
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
fprintf('  ÖNERİLEN ZIEGLER-NICHOLS KATSAYILARI\n');
fprintf('--------------------------------------------------------\n');
fprintf(' [1] Klasik Z-N (Agresif):\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n\n', zn_Kp, zn_Ki, zn_Kd);
fprintf(' [2] Pessen Integral Kuralı (Hızlı Tepki):\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n\n', pessen_Kp, pessen_Ki, pessen_Kd);
fprintf(' [3] Biraz Sönümlü (Some Overshoot):\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n\n', so_Kp, so_Ki, so_Kd);
fprintf(' [4] Overshootsuz (No Overshoot):\n');
fprintf('     Kp: %.3f | Ki: %.3f | Kd: %.3f\n', no_Kp, no_Ki, no_Kd);
fprintf('========================================================\n');
