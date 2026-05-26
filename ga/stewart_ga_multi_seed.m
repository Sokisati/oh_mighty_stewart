function [best_Kp, best_Ki, best_Kd] = stewart_ga_multi_seed(num_scenarios, pop_size, generations, show_plot, rng_seed)
%% stewart_ga_multi_seed.m
%  Robust Genetic Algorithm: Trains PID across multiple random wind/noise scenarios
%  (domain randomization)

if nargin < 1, num_scenarios = 100; end
if nargin < 2, pop_size = 30; end
if nargin < 3, generations = 50; end
if nargin < 4, show_plot = true; end
if nargin < 5, rng_seed = 42; end

deg2rad = pi / 180;

clc;
if show_plot, close all; end

fprintf('\n================================================\n');
fprintf('  STAGE 3: ROBUST GENETIC ALGORITHM PID OPTIMIZATION\n');
fprintf('================================================\n');
fprintf('Training config: Scenarios = %d, Pop Size = %d, Generations = %d\n', num_scenarios, pop_size, generations);

run('stewart_setup.m');
h0_val = h0;
r_limit_val = r_limit;
g_acc_val = g_acc;
c_roll_val = c_roll;


% Respect wind parameters set by RUN_ME.m, or default to robust 'realistic' wind if run directly
global WIND_TYPE;
global WIND_C_RATIO;
global WIND_R_RATIO;

if isempty(WIND_TYPE)
    WIND_TYPE = 'realistic';
end

if strcmpi(WIND_TYPE, 'combined')
    fprintf('Active Wind Scenario: COMBINED (c:%.2f r:%.2f)\n', WIND_C_RATIO, WIND_R_RATIO);
else
    fprintf('Active Wind Scenario: %s\n', upper(WIND_TYPE));
end

% 1. Pre-generate different scenarios (Domain Randomization)
fprintf('Pre-generating %d random wind and noise scenarios (Seed = %d)...\n', num_scenarios, rng_seed);
rng(rng_seed);
seeds = randi([1, 10000], 1, num_scenarios);

disturbances = cell(num_scenarios, 1);
noises = cell(num_scenarios, 1);

for s = 1:num_scenarios
    disturbances{s} = generate_disturbances(seeds(s), 30.0);
    noises{s} = generate_sensor_noise(seeds(s), 30.0, 0.02);
end
fprintf('Scenarios generated. Starting evolution...\n\n');

% 2. GA Parameters
POP_SIZE = pop_size;         
GENERATIONS = generations;      
MUTATION_IMPACT = 1.0; % Increased from 0.5 to allow larger jumps
FAILURE_THRESHOLD = 5000; % Threshold to filter out fell_off results

LB = [0,   0,    0];
UB = [25,  40,   10];  % Increased Kp, Ki, Kd bounds for more aggressive tuning

% 3. Initialize Random Population
pop = zeros(POP_SIZE, 3);
for i = 1:POP_SIZE
    pop(i,:) = LB + rand(1,3) .* (UB - LB);
end

fitness_scores = zeros(POP_SIZE, 1);
best_fitness_history = zeros(GENERATIONS, 1);
avg_fitness_history = zeros(GENERATIONS, 1);

% 4. Live Plotting
if show_plot
    fig = figure('Name', 'Robust GA Evolution', 'Color', [0.1 0.1 0.12], 'Position', [200 200 800 500]);
    ax = axes('Parent', fig, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w');
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, sprintf('Robust Evolution (Best Score across %d Scenarios)', num_scenarios), 'Color', 'w', 'FontSize', 12);
    xlabel(ax, 'Generation', 'Color', 'w');
    ylabel(ax, 'Average Benchmark Score', 'Color', 'w');
    h_best = plot(ax, NaN, NaN, 'g.-', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', 'Best Fitness');
    h_avg  = plot(ax, NaN, NaN, 'y.--', 'LineWidth', 1, 'MarkerSize', 10, 'DisplayName', 'Population Average');
    legend(ax, 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
end

max_tilt_rad = 30 * deg2rad;

for gen = 1:GENERATIONS
    % A) Evaluate Fitness across all scenarios
    parfor i = 1:POP_SIZE
        fitness_scores(i) = evaluate_fitness_robust(pop(i,:), h0_val, r_limit_val, g_acc_val, c_roll_val, max_tilt_rad, disturbances, noises);
    end
    
    [fitness_scores, sort_idx] = sort(fitness_scores);
    pop = pop(sort_idx, :);
    
    best_fitness = fitness_scores(1);
    survivors = fitness_scores(fitness_scores < FAILURE_THRESHOLD);
    if isempty(survivors), avg_fitness = best_fitness; else, avg_fitness = mean(survivors); end
    
    best_fitness_history(gen) = best_fitness;
    avg_fitness_history(gen)  = avg_fitness;
    
    fprintf('Gen %2d | Best Score: %6.2f | Elite Genes -> Kp: %4.2f, Ki: %4.2f, Kd: %4.2f\n', ...
        gen, 100 - best_fitness, pop(1,1), pop(1,2), pop(1,3));
        
    if show_plot && ishandle(fig)
        % Plot the actual scores (100 - fitness) instead of fitness
        set(h_best, 'XData', 1:gen, 'YData', 100 - best_fitness_history(1:gen));
        set(h_avg,  'XData', 1:gen, 'YData', 100 - avg_fitness_history(1:gen));
        xlim(ax, [1 GENERATIONS]);
        ylim(ax, [0 max(100 - best_fitness_history(1:gen)) * 1.2]);
        drawnow;
    end
    
    if gen == GENERATIONS, break; end
    
    % Less hasty early stopping: only stop if improvement is less than 0.001 over 20 generations
    if gen > 20
        if (best_fitness_history(gen-20) - best_fitness) < 0.001
            fprintf('Early stopping triggered at generation %d (No significant improvement, improvement < 0.001 in 20 generations).\n', gen);
            break;
        end
    end
    
    new_pop = zeros(POP_SIZE, 3);
    new_pop(1,:) = pop(1,:);
    new_pop(2,:) = pop(2,:);
    
    % Adaptive mutation rate: explores early (0.43), exploits late (0.08)
    mut_rate = 0.35 * (1 - gen/GENERATIONS) + 0.08;
    
    for i = 3:POP_SIZE
        % Tournament selection (k=2) from full population
        p1_idx = min(randi([1, POP_SIZE], 1, 2));
        p2_idx = min(randi([1, POP_SIZE], 1, 2));
        
        p1 = pop(p1_idx, :); p2 = pop(p2_idx, :);
        
        alpha = rand();
        child = alpha * p1 + (1 - alpha) * p2;
        
        for g = 1:3
            if rand() < mut_rate
                % Scale mutation impact by parameter range
                child(g) = child(g) + randn() * MUTATION_IMPACT * (UB(g) - LB(g)) / 10;
            end
            
            % Reflective bounds
            if child(g) > UB(g)
                child(g) = UB(g) - (child(g) - UB(g));
            elseif child(g) < LB(g)
                child(g) = LB(g) + (LB(g) - child(g));
            end
            % Safety clip
            child(g) = max(LB(g), min(UB(g), child(g)));
        end
        new_pop(i,:) = child;
    end
    pop = new_pop;
end

best_Kp = pop(1,1);
best_Ki = pop(1,2);
best_Kd = pop(1,3);

fprintf('\n================================================\n');
fprintf('ROBUST EVOLUTION COMPLETE!\n');
fprintf('Optimal PID Parameters Found (Survived %d Scenarios):\n', num_scenarios);
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.3f\n', best_Ki);
fprintf('  Kd = %.3f\n', best_Kd);
fprintf('================================================\n');

end
