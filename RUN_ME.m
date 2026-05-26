%% 
%% RUN_ME.m
%  ============================================================
%  STEWART PLATFORM - MAIN ENTRY POINT

clc;
fprintf('\n');
fprintf('================================================\n');
fprintf('       STEWART PLATFORM SIMULATION\n');
fprintf('       6-Leg Gough-Stewart Type\n');
fprintf('================================================\n\n');

% Add this folder to the MATLAB path
this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(this_dir));
cd(this_dir);

%% Mode selection
fprintf('Run mode:\n');
fprintf('  [1] PID Ball Balancing      (closed-loop, classic PID)\n');
fprintf('  [2] Manual Control          (arrows + WASD, game mode)\n');
fprintf('  [3] Robust GA PID Tuner     (Train 1 PID across 100 Scenarios)\n');
fprintf('  [4] The Ultimate Benchmark  (Compare Methods on 100 Seeds)\n');
fprintf('  [5] Wind Analyzer           (Realistic vs Chaotic Wind)\n');
fprintf('  [6] Ziegler-Nichols Auto-Tuner (Find Ku/Tu & Plot Oscillations)\n');
fprintf('  [7] MRAC Adaptive PID Simulation (Energy-based with leakage)\n');
fprintf('  [8] Lyapunov Adaptive PID Simulation (Sliding surface with leakage)\n\n');

choice = input('Enter choice (1-8, Enter = 1): ', 's');
if isempty(choice), choice = '1'; end

% If not running the benchmark, wind analyzer, or Z-N tuner, ask which wind type to use
if ~strcmp(choice, '4') && ~strcmp(choice, '5') && ~strcmp(choice, '6')
    % Default Classic Scenario ratios (loaded from config.txt)
    config = load_config();
    classic_c_ratio = config.classic_c_ratio;
    classic_r_ratio = config.classic_r_ratio;

    fprintf('\nSelect Wind Type for Simulation:\n');
    fprintf('  [1] Chaotic Wind   (Swirling, random directions)\n');
    fprintf('  [2] Realistic Wind (Consistent main direction)\n');
    fprintf('  [3] Combined Wind  (Blend of Chaotic and Realistic)\n');
    fprintf('  [4] Classic Scenario (%.2f Chaotic + %.2f Realistic)\n', classic_c_ratio, classic_r_ratio);
    w_choice = input('Enter choice (1-4, Enter = 4): ', 's');
    if isempty(w_choice), w_choice = '4'; end
    
    global WIND_TYPE;
    global WIND_C_RATIO;
    global WIND_R_RATIO;
    
    if strcmp(w_choice, '2')
        WIND_TYPE = 'realistic';
    elseif strcmp(w_choice, '3')
        WIND_TYPE = 'combined';
        c_str = input('Enter chaotic coefficient (0.0 to 1.0, default 0.3): ', 's');
        if isempty(c_str), WIND_C_RATIO = 0.3; else, WIND_C_RATIO = str2double(c_str); end
        r_str = input('Enter realistic coefficient (0.0 to 1.0, default 1.0): ', 's');
        if isempty(r_str), WIND_R_RATIO = 1.0; else, WIND_R_RATIO = str2double(r_str); end
    elseif strcmp(w_choice, '4')
        WIND_TYPE = 'combined';
        WIND_C_RATIO = classic_c_ratio;
        WIND_R_RATIO = classic_r_ratio;
    else
        WIND_TYPE = 'chaotic';
    end
end

fprintf('\n');

switch choice
    case '1'
        fprintf('[->] Launching PID ball balancing simulation...\n\n');
        run('stewart_pid_sim.m');

    case '2'
        fprintf('[->] Launching manual control (game mode)...\n\n');
        run('stewart_manual.m');

    case '3'
        fprintf('[->] Launching Robust Genetic Algorithm...\n\n');
        run('ga/stewart_ga_multi_seed.m');
        
    case '4'
        fprintf('[->] Launching The Ultimate Benchmark...\n\n');
        run('stewart_final_benchmark.m');
        
    case '5'
        fprintf('[->] Launching Wind Analyzer...\n\n');
        run('benchmarks/stewart_wind_analyzer.m');
        
    case '6'
        fprintf('[->] Launching Ziegler-Nichols Auto-Tuner...\n\n');
        run('tuning/stewart_zn_tuner.m');

    case '7'
        fprintf('[->] Launching MRAC Adaptive PID Simulation...\n\n');
        run('simulations/stewart_mrac_sim.m');

    case '8'
        fprintf('[->] Launching Lyapunov Adaptive PID Simulation...\n\n');
        run('simulations/stewart_lyap_sim.m');

    otherwise
        fprintf('[!] Invalid choice. Defaulting to PID ball balancing simulation.\n');
        run('stewart_pid_sim.m');
end
