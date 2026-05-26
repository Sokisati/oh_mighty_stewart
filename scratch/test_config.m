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

fprintf('=== Verification complete! ===\n');
