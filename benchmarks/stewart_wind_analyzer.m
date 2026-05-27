
clc; close all;
fprintf('\n====================================================\n');
fprintf('     WIND ANALYZER: Chaotic vs Realistic\n');
fprintf('====================================================\n\n');

seed_str = input('Enter random seed for wind generation (integer, default 1): ', 's');
if isempty(seed_str), seed = 1; else, seed = str2double(seed_str); end

c_str = input('Enter chaotic wind coefficient (0.0 to 1.0, default 0.3): ', 's');
if isempty(c_str), c_ratio = 0.3; else, c_ratio = str2double(c_str); end

r_str = input('Enter realistic wind coefficient (0.0 to 1.0, default 1.0): ', 's');
if isempty(r_str), r_ratio = 1.0; else, r_ratio = str2double(r_str); end

T_sim = 30;

fprintf('Generating 30 seconds of wind data...\n');
chaotic_wind = generate_chaotic_disturbances(seed, T_sim);
realistic_wind = generate_realistic_disturbances(seed, T_sim);
combined_wind = generate_combined_disturbances(seed, c_ratio, r_ratio, T_sim);

fprintf('  Chaotic   : %d kicks generated.\n', size(chaotic_wind, 1));
fprintf('  Realistic : %d kicks generated.\n', size(realistic_wind, 1));
fprintf('  Combined  : %d kicks generated.\n\n', size(combined_wind, 1));

c_red = [1.0, 0.2, 0.2];
c_yel = [1.0, 0.8, 0.1];
c_blu = [0.1, 0.6, 1.0];
half1 = [linspace(c_red(1), c_yel(1), 32)', linspace(c_red(2), c_yel(2), 32)', linspace(c_red(3), c_yel(3), 32)'];
half2 = [linspace(c_yel(1), c_blu(1), 32)', linspace(c_yel(2), c_blu(2), 32)', linspace(c_yel(3), c_blu(3), 32)'];
cmap = [half1; half2];

fig1 = figure('Name', 'Chaotic Wind Analysis', 'Color', [0.10 0.10 0.13], ...
              'NumberTitle', 'off', 'Position', [50, 100, 800, 800]);

num_kicks_c = size(chaotic_wind, 1);

ax1_top = subplot(2, 1, 1);
set(ax1_top, 'Color', [0.08 0.08 0.10], 'GridColor', [0.4 0.4 0.4], ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
hold on; grid on; axis equal;
xline(0, '--', 'Color', [0.5 0.5 0.5]);
yline(0, '--', 'Color', [0.5 0.5 0.5]);
xlim([-0.5 0.5]); ylim([-0.5 0.5]);
xlabel('Wind v_x [m/s]', 'Color', [0.8 0.8 0.8]); 
ylabel('Wind v_y [m/s]', 'Color', [0.8 0.8 0.8]);
title('Chaotic Wind - Vector Map (Origin Arrows)', 'Color', [0.95 0.95 0.95], 'FontWeight', 'bold');

ax1_bot = subplot(2, 1, 2);
set(ax1_bot, 'Color', [0.08 0.08 0.10], 'GridColor', [0.4 0.4 0.4], ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
hold on; grid on;
xlim([0 T_sim]);
ylim([0 0.5]);
xlabel('Time [s]', 'Color', [0.8 0.8 0.8]);
ylabel('Wind Magnitude [m/s]', 'Color', [0.8 0.8 0.8]);
title('Chaotic Wind - Timeline', 'Color', [0.95 0.95 0.95], 'FontWeight', 'bold');

for i = 1:num_kicks_c
    t  = chaotic_wind(i, 1);
    vx = chaotic_wind(i, 2);
    vy = chaotic_wind(i, 3);
    mag = sqrt(vx^2 + vy^2);
    
    color_idx = max(1, min(64, round((t / T_sim) * 64)));
    c = cmap(color_idx, :);
    
    quiver(ax1_top, 0, 0, vx, vy, 0, 'Color', c, 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    plot(ax1_bot, [t, t], [0, mag], 'Color', c, 'LineWidth', 2);
    plot(ax1_bot, t, mag, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', c, 'MarkerSize', 5);
end

colormap(fig1, cmap);
cb1 = colorbar(ax1_top, 'eastoutside');
caxis(ax1_top, [0 T_sim]);
cb1.Color = [0.8 0.8 0.8];
cb1.Label.String = 'Time [s]';
cb1.Label.Color = [0.8 0.8 0.8];

cb2 = colorbar(ax1_bot, 'eastoutside');
caxis(ax1_bot, [0 T_sim]);
cb2.Color = [0.8 0.8 0.8];
cb2.Label.String = 'Time [s]';
cb2.Label.Color = [0.8 0.8 0.8];

fig2 = figure('Name', 'Realistic Wind Analysis', 'Color', [0.10 0.10 0.13], ...
              'NumberTitle', 'off', 'Position', [900, 100, 800, 800]);

num_kicks_r = size(realistic_wind, 1);

ax2_top = subplot(2, 1, 1);
set(ax2_top, 'Color', [0.08 0.08 0.10], 'GridColor', [0.4 0.4 0.4], ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
hold on; grid on; axis equal;
xline(0, '--', 'Color', [0.5 0.5 0.5]);
yline(0, '--', 'Color', [0.5 0.5 0.5]);
xlim([-0.5 0.5]); ylim([-0.5 0.5]);
xlabel('Wind v_x [m/s]', 'Color', [0.8 0.8 0.8]); 
ylabel('Wind v_y [m/s]', 'Color', [0.8 0.8 0.8]);
title('Realistic Wind - Vector Map (Origin Arrows)', 'Color', [0.95 0.95 0.95], 'FontWeight', 'bold');

ax2_bot = subplot(2, 1, 2);
set(ax2_bot, 'Color', [0.08 0.08 0.10], 'GridColor', [0.4 0.4 0.4], ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
hold on; grid on;
xlim([0 T_sim]);
ylim([0 0.5]);
xlabel('Time [s]', 'Color', [0.8 0.8 0.8]);
ylabel('Wind Magnitude [m/s]', 'Color', [0.8 0.8 0.8]);
title('Realistic Wind - Timeline', 'Color', [0.95 0.95 0.95], 'FontWeight', 'bold');

for i = 1:num_kicks_r
    t  = realistic_wind(i, 1);
    vx = realistic_wind(i, 2);
    vy = realistic_wind(i, 3);
    mag = sqrt(vx^2 + vy^2);
    
    color_idx = max(1, min(64, round((t / T_sim) * 64)));
    c = cmap(color_idx, :);
    
    quiver(ax2_top, 0, 0, vx, vy, 0, 'Color', c, 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    plot(ax2_bot, [t, t], [0, mag], 'Color', c, 'LineWidth', 2);
    plot(ax2_bot, t, mag, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', c, 'MarkerSize', 5);
end

colormap(fig2, cmap);
cb3 = colorbar(ax2_top, 'eastoutside');
caxis(ax2_top, [0 T_sim]);
cb3.Color = [0.8 0.8 0.8];
cb3.Label.String = 'Time [s]';
cb3.Label.Color = [0.8 0.8 0.8];

cb4 = colorbar(ax2_bot, 'eastoutside');
caxis(ax2_bot, [0 T_sim]);
cb4.Color = [0.8 0.8 0.8];
cb4.Label.String = 'Time [s]';
cb4.Label.Color = [0.8 0.8 0.8];

fig3 = figure('Name', 'Combined Wind Analysis', 'Color', [0.10 0.10 0.13], ...
              'NumberTitle', 'off', 'Position', [100, 100, 800, 800]);

num_kicks_comb = size(combined_wind, 1);

ax3_top = subplot(2, 1, 1);
set(ax3_top, 'Color', [0.08 0.08 0.10], 'GridColor', [0.4 0.4 0.4], ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
hold on; grid on; axis equal;
xline(0, '--', 'Color', [0.5 0.5 0.5]);
yline(0, '--', 'Color', [0.5 0.5 0.5]);
xlim([-0.5 0.5]); ylim([-0.5 0.5]);
xlabel('Wind v_x [m/s]', 'Color', [0.8 0.8 0.8]); 
ylabel('Wind v_y [m/s]', 'Color', [0.8 0.8 0.8]);
title(sprintf('Combined Wind (c=%.2f, r=%.2f) - Vector Map', c_ratio, r_ratio), 'Color', [0.95 0.95 0.95], 'FontWeight', 'bold');

ax3_bot = subplot(2, 1, 2);
set(ax3_bot, 'Color', [0.08 0.08 0.10], 'GridColor', [0.4 0.4 0.4], ...
    'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8]);
hold on; grid on;
xlim([0 T_sim]);
ylim([0 0.5]);
xlabel('Time [s]', 'Color', [0.8 0.8 0.8]);
ylabel('Wind Magnitude [m/s]', 'Color', [0.8 0.8 0.8]);
title('Combined Wind - Timeline', 'Color', [0.95 0.95 0.95], 'FontWeight', 'bold');

for i = 1:num_kicks_comb
    t  = combined_wind(i, 1);
    vx = combined_wind(i, 2);
    vy = combined_wind(i, 3);
    mag = sqrt(vx^2 + vy^2);
    
    color_idx = max(1, min(64, round((t / T_sim) * 64)));
    c = cmap(color_idx, :);
    
    quiver(ax3_top, 0, 0, vx, vy, 0, 'Color', c, 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    plot(ax3_bot, [t, t], [0, mag], 'Color', c, 'LineWidth', 2);
    plot(ax3_bot, t, mag, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', c, 'MarkerSize', 5);
end

colormap(fig3, cmap);
cb5 = colorbar(ax3_top, 'eastoutside');
caxis(ax3_top, [0 T_sim]);
cb5.Color = [0.8 0.8 0.8]; cb5.Label.String = 'Time [s]'; cb5.Label.Color = [0.8 0.8 0.8];

cb6 = colorbar(ax3_bot, 'eastoutside');
caxis(ax3_bot, [0 T_sim]);
cb6.Color = [0.8 0.8 0.8]; cb6.Label.String = 'Time [s]'; cb6.Label.Color = [0.8 0.8 0.8];

fprintf('Visualizations are ready on the figure windows.\n\n');
