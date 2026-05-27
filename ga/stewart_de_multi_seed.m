function [best_Kp, best_Ki, best_Kd] = stewart_de_multi_seed(num_scenarios, pop_size, generations, show_plot, rng_seed)
%% stewart_de_multi_seed.m
%  Robust Differential Evolution (DE/rand/1/bin)

if nargin < 1, num_scenarios = 100; end
if nargin < 2, pop_size = 30; end
if nargin < 3, generations = 50; end
if nargin < 4, show_plot = true; end
if nargin < 5, rng_seed = 42; end

deg2rad = pi / 180;

clc;
if show_plot, close all; end

fprintf('\n================================================\n');
fprintf('  ROBUST DIFFERENTIAL EVOLUTION (DE/rand/1/bin)\n');
fprintf('================================================\n');
fprintf('Training config: Scenarios = %d, Pop Size = %d, Generations = %d\n', num_scenarios, pop_size, generations);

run('stewart_setup.m');
h0_val = h0;
r_limit_val = r_limit;
g_acc_val = g_acc;
c_roll_val = c_roll;

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

POP_SIZE = pop_size;         
GENERATIONS = generations;      
F_scale = 0.8;
CR = 0.9;
FAILURE_THRESHOLD = 5000;

UB = [25.0, 40.0, 10.0];
LB = [0.0,  0.0,  0.0];

% Initialize Random Population
pop = zeros(POP_SIZE, 3);
for i = 1:POP_SIZE
    pop(i,:) = LB + rand(1,3) .* (UB - LB);
end

fitness_scores = zeros(POP_SIZE, 1);
robust_scores = zeros(POP_SIZE, 1);
num_drops = zeros(POP_SIZE, 1);

best_fitness_history = zeros(GENERATIONS, 1);
avg_fitness_history = zeros(GENERATIONS, 1);
best_score_history = zeros(GENERATIONS, 1);
avg_score_history = zeros(GENERATIONS, 1);

if show_plot
    fig = figure('Name', 'Robust DE Evolution', 'Color', [0.1 0.1 0.12], 'Position', [200 200 800 500]);
    ax = axes('Parent', fig, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w');
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, sprintf('Robust DE Evolution (Best Score across %d Scenarios)', num_scenarios), 'Color', 'w', 'FontSize', 12);
    xlabel(ax, 'Generation', 'Color', 'w');
    ylabel(ax, 'Average Benchmark Score', 'Color', 'w');
    h_best = plot(ax, NaN, NaN, 'g.-', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', 'Best Fitness');
    h_avg  = plot(ax, NaN, NaN, 'y.--', 'LineWidth', 1, 'MarkerSize', 10, 'DisplayName', 'Population Average');
    legend(ax, 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
end

max_tilt_rad = 30 * deg2rad;

for gen = 1:GENERATIONS
    rng(rng_seed + gen * 100);
    seeds = randi([1, 100000], 1, num_scenarios);
    disturbances = cell(num_scenarios, 1);
    noises = cell(num_scenarios, 1);
    for s = 1:num_scenarios
        disturbances{s} = generate_disturbances(seeds(s), 30.0);
        noises{s} = generate_sensor_noise(seeds(s), 30.0, 0.02);
    end

    parfor i = 1:POP_SIZE
        [fitness_scores(i), robust_scores(i), num_drops(i)] = evaluate_fitness_robust(pop(i,:), h0_val, r_limit_val, g_acc_val, c_roll_val, max_tilt_rad, disturbances, noises);
    end
    
    [fitness_scores, sort_idx] = sort(fitness_scores);
    pop = pop(sort_idx, :);
    robust_scores = robust_scores(sort_idx);
    num_drops = num_drops(sort_idx);
    
    best_fitness = fitness_scores(1);
    survivors = fitness_scores(fitness_scores < FAILURE_THRESHOLD);
    if isempty(survivors), avg_fitness = best_fitness; else, avg_fitness = mean(survivors); end
    
    best_robust = robust_scores(1);
    best_drops  = num_drops(1);
    
    best_fitness_history(gen) = best_fitness;
    avg_fitness_history(gen)  = avg_fitness;
    best_score_history(gen)   = best_robust;
    avg_score_history(gen)    = mean(robust_scores);
    
    fprintf('Gen %2d | Best Score: %6.2f | Drops: %2d | Elite Genes -> Kp: %4.2f, Ki: %4.2f, Kd: %4.2f\n', ...
        gen, best_robust, best_drops, pop(1,1), pop(1,2), pop(1,3));
        
    if show_plot && ishandle(fig)
        set(h_best, 'XData', 1:gen, 'YData', best_score_history(1:gen));
        set(h_avg,  'XData', 1:gen, 'YData', avg_score_history(1:gen));
        xlim(ax, [1 GENERATIONS]);
        ylim(ax, [0 max(best_score_history(1:gen)) * 1.2]);
        drawnow;
    end
    
    if gen == GENERATIONS, break; end
    
    if gen > 20
        if (best_fitness_history(gen-20) - best_fitness) < 0.001
            fprintf('Early stopping triggered at generation %d.\n', gen);
            break;
        end
    end
    
    new_pop = pop;
    for i = 1:POP_SIZE
        idx = randperm(POP_SIZE, 3);
        while any(idx == i)
            idx = randperm(POP_SIZE, 3);
        end
        r1 = idx(1); r2 = idx(2); r3 = idx(3);
        
        v = pop(r1, :) + F_scale * (pop(r2, :) - pop(r3, :));
        
        for g = 1:3
            v(g) = max(LB(g), min(UB(g), v(g)));
        end
        
        j_rand = randi([1, 3]);
        u = pop(i, :);
        for g = 1:3
            if rand() <= CR || g == j_rand
                u(g) = v(g);
            end
        end
        
        new_pop(i, :) = u;
    end
    
    % Optional: evaluate candidates and only keep if better (standard DE logic)
    % For multi-scenario robust optimization, since fitness landscape is noisy and changes slightly, 
    % we evaluate all in the next generation. So we just set pop = new_pop.
    % To preserve elitism, we guarantee the best stays.
    new_pop(1,:) = pop(1,:); 
    pop = new_pop;
end

best_Kp = pop(1,1);
best_Ki = pop(1,2);
best_Kd = pop(1,3);

fprintf('\n================================================\n');
fprintf('ROBUST DE EVOLUTION COMPLETE!\n');
fprintf('Optimal PID Parameters Found:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.3f\n', best_Ki);
fprintf('  Kd = %.3f\n', best_Kd);
fprintf('================================================\n');

end
