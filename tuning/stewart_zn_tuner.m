% stewart_zn_tuner.m
% Automates Ziegler-Nichols tuning for the Stewart Platform
% Uses binary search to find Ultimate Gain (Ku) and Ultimate Period (Tu).
% Plots the constant oscillation for visual verification.

clc; close all;

% Add core folder to path
this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(this_dir, '..', 'core')));
addpath(genpath(fullfile(this_dir, '..', 'environment')));

fprintf('========================================================\n');
fprintf('       ZIEGLER-NICHOLS OTONOM TUNING MODÜLÜ\n');
fprintf('========================================================\n\n');

%% 1. Fiziksel Parametreler
params.dt       = 0.01;
params.T_sim    = 20.0; % Daha uzun simülasyon, salınımları net görmek için
params.g_acc    = 9.81;
params.c_roll   = 0.01;
params.r_limit  = 0.20; % 20 cm tepsi yarıçapı
params.max_tilt = 30 * (pi / 180);

% Güncel fiziksel limitlerimiz (Slew Rate & Delay)
params.delay_sec = 0.020; % 20 ms
params.slew_rate = 350 * (pi / 180); % 350 deg/s

% Z-N testi için topu merkez dışından başlat
params.ball_x0 = 0.05; 
params.ball_y0 = 0.0;

% Rüzgar ve gürültü YOK
disturb_table = [];
noise_table   = [];

%% 2. Binary Search ile Ku (Ultimate Gain) Bulma
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
    
    % Z-N Kuralları: Sadece Oransal (P) kontrol devrede
    Ki_test = 0;
    Kd_test = 0;
    
    res = simulate_ball(Kp_test, Ki_test, Kd_test, params, disturb_table, noise_table);
    
    if res.fell_off
        % Top düştüyse sistem bariz şekilde kararsızdır
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Düştü (Kararsız) -> Kp azaltılıyor\n', iter, Kp_test);
        Kp_high = Kp_test;
        continue;
    end
    
    % Top düşmediyse salınım genliğini (tepe noktalarını) analiz et
    % 2D radyal uzaklık kullan: sqrt(x^2 + y^2) — tek eksen analizinden daha güvenilir
    r_pos = sqrt(res.ball_x.^2 + res.ball_y.^2);
    
    % Radyal uzaklığın tepe noktalarını bul (her iki yöndeki salınımı yakalar)
    idx_peaks = find(r_pos(2:end-1) > r_pos(1:end-2) & r_pos(2:end-1) > r_pos(3:end)) + 1;
    
    % Sadece 2. saniyeden sonraki tepelere bak (geçici başlangıç dinamiklerini atla)
    t_peaks = res.t_vec(idx_peaks);
    valid_mask = t_peaks > 2.0;
    idx_peaks = idx_peaks(valid_mask);
    t_peaks = t_peaks(valid_mask);
    
    if length(idx_peaks) < 5
        % Yeterince salınım yok, sistem çok sönümlü
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Aşırı Sönümlü (Salınım yok) -> Kp artırılıyor\n', iter, Kp_test);
        Kp_low = Kp_test;
        continue;
    end
    
    % Son 4 tepeden genlik değişimini hesapla (daha güvenilir istatistik)
    peaks_amp = r_pos(idx_peaks(end-3:end));
    % Lineer regresyon ile büyüme trendi: pozitif eğim = büyüyor, negatif = sönümleniyor
    t_local = (1:4)';
    slope = (t_local' * peaks_amp - mean(t_local) * sum(peaks_amp)) / (t_local' * t_local - 4 * mean(t_local)^2);
    norm_slope = slope / mean(peaks_amp); % normalize edilmiş eğim
    
    % Periyot: son birkaç tepe arasındaki ortalama aralık
    Tu_est = mean(diff(t_peaks(end-3:end)));
    
    % Tolerans: normalize eğim |< 0.01/s (yaklaşık sabit genlik)
    tol = 0.015;
    if norm_slope > tol
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Salınım Büyüyor (slope=%.4f) -> Kp azaltılıyor\n', iter, Kp_test, norm_slope);
        Kp_high = Kp_test;
        % Ku = büyümeye geçiş noktası (üst sınır), gerçek Ku buraya yakın
        Ku = Kp_test;
        Tu = Tu_est;
        best_res = res;
        best_idx_peaks = idx_peaks;
    elseif norm_slope < -tol
        fprintf('Iter %2d | Kp = %6.3f | SONUÇ: Sönümleniyor (slope=%.4f) -> Kp artırılıyor\n', iter, Kp_test, norm_slope);
        Kp_low = Kp_test;
        % Sönümlenme tarafında Ku'yu güncelleme — sadece büyüme sınırını tut
        if Ku == 0
            % Henüz büyüme görülmediyse yine de kaydet (başlangıç için)
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

%% 3. Görselleştirme (Plotting)
if ~isempty(best_res)
    r_plot = sqrt(best_res.ball_x.^2 + best_res.ball_y.^2) * 100; % cm
    figure('Name', 'Ziegler-Nichols Kararlılık Sınırı (Ultimate Gain)', 'Position', [100, 100, 900, 500]);
    subplot(2,1,1);
    plot(best_res.t_vec, best_res.ball_x * 100, 'b-', 'LineWidth', 1.5); hold on;
    plot(best_res.t_vec, best_res.ball_y * 100, 'g-', 'LineWidth', 1.2);
    yline(0, 'k--', 'LineWidth', 0.8);
    title(sprintf('X ve Y Pozisyonları (Ku = %.3f)', Ku));
    xlabel('Zaman (s)'); ylabel('Pozisyon (cm)');
    legend('X', 'Y', 'Merkez', 'Location', 'best'); grid on;
    
    subplot(2,1,2);
    plot(best_res.t_vec, r_plot, 'r-', 'LineWidth', 1.5); hold on;
    plot(best_res.t_vec(best_idx_peaks), r_plot(best_idx_peaks), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    title(sprintf('Radyal Uzaklık — Ku = %.3f, Tu = %.3f s', Ku, Tu));
    xlabel('Zaman (s)'); ylabel('Uzaklık (cm)');
    legend('Radyal Uzaklık', 'Tepe Noktaları', 'Location', 'best'); grid on;
    drawnow;
end

%% 4. PID Parametrelerini Hesaplama ve Konsola Yazdırma
fprintf('\n========================================================\n');
fprintf('  SİSTEMİN FİZİKSEL LİMİTLERİ (Ku ve Tu)\n');
fprintf('--------------------------------------------------------\n');
fprintf(' Ultimate Gain (Ku)   : %.3f\n', Ku);
fprintf(' Ultimate Period (Tu) : %.3f saniye\n', Tu);
fprintf('========================================================\n\n');

% Z-N Klasik PID
zn_Kp = 0.6 * Ku;
zn_Ki = 1.2 * Ku / Tu;
zn_Kd = 0.075 * Ku * Tu;

% Pessen Integral
pessen_Kp = 0.7 * Ku;
pessen_Ki = 1.75 * Ku / Tu;
pessen_Kd = 0.105 * Ku * Tu;

% Biraz Sönümlü
so_Kp = 0.33 * Ku;
so_Ki = 0.66 * Ku / Tu;
so_Kd = 0.11 * Ku * Tu;

% Overshootsuz
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
