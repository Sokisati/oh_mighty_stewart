
clc;
fprintf('\n');
fprintf('================================================\n');
fprintf('       STEWART PLATFORM SIMULATION\n');
fprintf('       6-Leg Gough-Stewart Type\n');
fprintf('================================================\n\n');

this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(this_dir));
cd(this_dir);

clc;
fprintf('\n');
fprintf('================================================\n');
fprintf('       STEWART PLATFORM SIMULATION\n');
fprintf('       6-Leg Gough-Stewart Type\n');
fprintf('================================================\n\n');

this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(this_dir));
cd(this_dir);

fprintf('Run mode:\n');
fprintf('  [1] PID Ball Balancing      \n');
fprintf('  [2] Manual Control          \n');
fprintf('  [3] Wind Analyzer           \n');
fprintf('  [4] Ziegler-Nichols Auto-Tuner \n');
fprintf('  [5] Evolutionary Benchmark \n');
fprintf('  [6] Adaptive PID Benchmark  \n');
fprintf('  [7] Ultimate Benchmark      \n\n');
fprintf('  Press any other number to exit program \n');

choice = input('Enter choice (1-7, Enter = 1): ', 's');
if isempty(choice), choice = '1'; end

valid_choices = {'1', '2', '3', '4', '5', '6', '7'};
if ~ismember(choice, valid_choices)
    fprintf('Exiting program.\n');
    return;
end

if strcmp(choice, '1')
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
        run('simulations/stewart_pid_sim.m');

    case '2'
        fprintf('[->] Launching manual control (game mode)...\n\n');
        run('simulations/stewart_manual.m');

    case '3'
        fprintf('[->] Launching Wind Analyzer...\n\n');
        run('benchmarks/stewart_wind_analyzer.m');
        
    case '4'
        fprintf('[->] Launching Ziegler-Nichols Auto-Tuner...\n\n');
        run('tuning/stewart_zn_tuner.m');

    case '5'
        fprintf('[->] Launching Evolutionary Algorithms Comparative Benchmark...\n\n');
        run('benchmarks/stewart_comparative_benchmark.m');
        
    case '6'
        fprintf('[->] Launching Adaptive PID Benchmark...\n\n');
        run('benchmarks/stewart_adaptive_benchmark.m');
        
    case '7'
        fprintf('[->] Launching Ultimate Benchmark...\n\n');
        run('benchmarks/stewart_ultimate_benchmark.m');

    otherwise
        fprintf('Exiting program.\n');
        return;
end

