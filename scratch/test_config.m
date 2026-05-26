%% test_config.m
%  Verify load_config loads variables correctly.

clc;
clear;

fprintf('=== Testing load_config utility ===\n');

% Add path dynamically so it can find utils
this_dir = fileparts(mfilename('fullpath'));
proj_dir = fileparts(this_dir);
addpath(genpath(proj_dir));

% Call config loader
load_config;

% Check a few variables
fprintf('R_base        = %.3f m (Expected: 0.200)\n', R_base);
fprintf('R_top         = %.3f m (Expected: 0.120)\n', R_top);
fprintf('h0            = %.3f m (Expected: 0.200)\n', h0);
fprintf('alpha_b_deg   = %.1f deg (Expected: 0.0)\n', alpha_b_deg);
fprintf('alpha_t_deg   = %.1f deg (Expected: 20.0)\n', alpha_t_deg);
fprintf('Kp            = %.1f (Expected: 8.0)\n', Kp);
fprintf('Ki            = %.1f (Expected: 1.0)\n', Ki);
fprintf('Kd            = %.1f (Expected: 0.9)\n', Kd);
fprintf('delay_sec     = %.3f s (Expected: 0.040)\n', delay_sec);
fprintf('slew_rate_deg = %.1f deg/s (Expected: 250.0)\n', slew_rate_deg);

% Disturbance & Noise Parameters
fprintf('real_min_interval = %.3f s (Expected: 0.280)\n', real_min_interval);
fprintf('real_max_interval = %.3f s (Expected: 0.360)\n', real_max_interval);
fprintf('real_min_force    = %.3f m/s (Expected: 0.300)\n', real_min_force);
fprintf('real_max_force    = %.3f m/s (Expected: 0.350)\n', real_max_force);
fprintf('real_angle_spread = %.1f deg (Expected: 10.0)\n', real_angle_spread);

fprintf('chao_min_interval = %.3f s (Expected: 1.800)\n', chao_min_interval);
fprintf('chao_max_interval = %.3f s (Expected: 3.000)\n', chao_max_interval);
fprintf('chao_min_force    = %.3f m/s (Expected: 0.450)\n', chao_min_force);
fprintf('chao_max_force    = %.3f m/s (Expected: 0.750)\n', chao_max_force);
fprintf('chao_angle_drift_speed = %.1f deg/s (Expected: 25.0)\n', chao_angle_drift_speed);
fprintf('chao_angle_spread = %.1f deg (Expected: 30.0)\n', chao_angle_spread);
fprintf('chao_anti_cancel_deg = %.1f deg (Expected: 45.0)\n', chao_anti_cancel_deg);

fprintf('noise_sigma_white = %.6f m (Expected: 0.000020)\n', noise_sigma_white);
fprintf('noise_sigma_rw    = %.6f m (Expected: 0.000010)\n', noise_sigma_rw);
fprintf('noise_spike_prob  = %.6f (Expected: 0.000100)\n', noise_spike_prob);
fprintf('noise_spike_mag   = %.6f m (Expected: 0.000100)\n', noise_spike_mag);

fprintf('classic_c_ratio   = %.2f (Expected: 0.60)\n', classic_c_ratio);
fprintf('classic_r_ratio   = %.2f (Expected: 1.00)\n', classic_r_ratio);

fprintf('=== Verification complete! ===\n');
