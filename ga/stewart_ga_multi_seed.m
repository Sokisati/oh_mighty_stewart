function [best_Kp, best_Ki, best_Kd] = stewart_ga_multi_seed(num_scenarios, pop_size, generations, show_plot, rng_seed)

config = load_config();
if nargin < 1, num_scenarios = 100; end
if nargin < 2, pop_size = config.ga_pop_size; end
if nargin < 3, generations = config.ga_generations; end
if nargin < 4, show_plot = true; end
if nargin < 5, rng_seed = 42; end

deg2rad = pi / 180;

clc;
if show_plot, close all; end

fprintf('\n================================================\n');
fprintf('  STAGE 3: ROBUST ISLAND MODEL GA PID TUNING\n');
fprintf('================================================\n');

NUM_ISLANDS = 4;
MIGRATION_INTERVAL = 10;
island_pop_size = max(4, ceil(pop_size / NUM_ISLANDS));
POP_SIZE = island_pop_size * NUM_ISLANDS; % Adjust total

fprintf('Training config: Scenarios = %d, Total Pop Size = %d (%d Islands x %d), Generations = %d\n', ...
    num_scenarios, POP_SIZE, NUM_ISLANDS, island_pop_size, generations);

run('stewart_setup.m');
h0_val = h0;
r_limit_val = r_limit;
g_acc_val = g_acc;
c_roll_val = c_roll;

global WIND_TYPE;
global WIND_C_RATIO;
global WIND_R_RATIO;

if isempty(WIND_TYPE), WIND_TYPE = 'realistic'; end

if strcmpi(WIND_TYPE, 'combined')
    fprintf('Active Wind Scenario: COMBINED (c:%.2f r:%.2f)\n', WIND_C_RATIO, WIND_R_RATIO);
else
    fprintf('Active Wind Scenario: %s\n', upper(WIND_TYPE));
end

MUTATION_IMPACT = 2.5; % Increased aggressively to explore further bounds 

UB = [25.0, 40.0, 10.0];
LB = [0.0,  0.0,  0.0];

pop = zeros(NUM_ISLANDS, island_pop_size, 3);
for i = 1:NUM_ISLANDS
    for j = 1:island_pop_size
        pop(i,j,:) = LB + rand(1,3) .* (UB - LB);
    end
end

best_fitness_history = zeros(generations, 1);
avg_fitness_history = zeros(generations, 1);
best_score_history = zeros(generations, 1);

if show_plot
    fig = figure('Name', 'Robust Island GA Evolution', 'Color', [0.1 0.1 0.12], 'Position', [200 200 800 500]);
    ax = axes('Parent', fig, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w');
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, sprintf('Island GA Evolution (Best Score across %d Scenarios)', num_scenarios), 'Color', 'w', 'FontSize', 12);
    xlabel(ax, 'Generation', 'Color', 'w');
    ylabel(ax, 'Global Best Benchmark Score', 'Color', 'w');
    h_best = plot(ax, NaN, NaN, 'g.-', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', 'Global Best Score');
    legend(ax, 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
end

rng(rng_seed);
seeds = randi([1, 100000], 1, num_scenarios);
disturbances = cell(num_scenarios, 1);
noises = cell(num_scenarios, 1);
for s = 1:num_scenarios
    disturbances{s} = generate_disturbances(seeds(s), 30.0);
    noises{s} = generate_sensor_noise(seeds(s), 30.0, 0.02);
end
fprintf('Generated %d fixed scenarios to create a deterministic fitness landscape.\n', num_scenarios);

max_tilt_rad = 30 * deg2rad;

for gen = 1:generations
    flat_pop = reshape(pop, [POP_SIZE, 3]);
    flat_fit = zeros(POP_SIZE, 1);
    flat_rob = zeros(POP_SIZE, 1);
    flat_drop = zeros(POP_SIZE, 1);
    
    parfor k = 1:POP_SIZE
        [flat_fit(k), flat_rob(k), flat_drop(k)] = evaluate_fitness_robust(flat_pop(k,:), h0_val, r_limit_val, g_acc_val, c_roll_val, max_tilt_rad, disturbances, noises);
    end
    
    fitness_scores = reshape(flat_fit, [NUM_ISLANDS, island_pop_size]);
    robust_scores = reshape(flat_rob, [NUM_ISLANDS, island_pop_size]);
    num_drops = reshape(flat_drop, [NUM_ISLANDS, island_pop_size]);
    
    new_pop = zeros(NUM_ISLANDS, island_pop_size, 3);
    
    global_best_fit = inf;
    global_best_idx = [1,1];
    global_best_rob = -inf;
    global_best_drops = inf;
    
    avg_fit = mean(flat_fit);
    
    for i = 1:NUM_ISLANDS
        [island_fit, sort_idx] = sort(fitness_scores(i, :));
        island_pop = squeeze(pop(i, sort_idx, :));
        if island_pop_size == 1, island_pop = reshape(island_pop, 1, 3); end
        island_rob = robust_scores(i, sort_idx);
        island_drops = num_drops(i, sort_idx);
        
        if island_fit(1) < global_best_fit
            global_best_fit = island_fit(1);
            global_best_idx = [i, 1];
            global_best_rob = island_rob(1);
            global_best_drops = island_drops(1);
        end
        
        new_island = zeros(island_pop_size, 3);
        new_island(1,:) = island_pop(1,:);
        new_island(2,:) = island_pop(2,:);
        
        mut_rate = 0.50 * (1 - gen/generations) + 0.10;
        
        for j = 3:island_pop_size
            p1_idx = min(randi([1, island_pop_size], 1, 2));
            p2_idx = min(randi([1, island_pop_size], 1, 2));
            p1 = island_pop(p1_idx, :); p2 = island_pop(p2_idx, :);
            
            alpha = rand();
            child = alpha * p1 + (1 - alpha) * p2;
            
            for g = 1:3
                if rand() < mut_rate
                    child(g) = child(g) + randn() * MUTATION_IMPACT * (UB(g) - LB(g)) / 10;
                end
                
                if child(g) > UB(g)
                    child(g) = UB(g) - (child(g) - UB(g));
                elseif child(g) < LB(g)
                    child(g) = LB(g) + (LB(g) - child(g));
                end
                child(g) = max(LB(g), min(UB(g), child(g)));
            end
            new_island(j,:) = child;
        end
        new_pop(i, :, :) = new_island;
        pop(i, :, :) = island_pop; % Update current with sorted for migration
    end
    
    if mod(gen, MIGRATION_INTERVAL) == 0 && gen ~= generations
        fprintf('  [MIGRATION] Islands exchanging elite individuals...\n');
        for i = 1:NUM_ISLANDS
            target = mod(i, NUM_ISLANDS) + 1; % Ring topology
            new_pop(target, end, :) = pop(i, 1, :); % Best of source replaces worst of target
        end
    end
    
    pop = new_pop;
    
    best_fitness_history(gen) = global_best_fit;
    avg_fitness_history(gen)  = avg_fit;
    best_score_history(gen)   = global_best_rob;
    
    best_Kp = pop(global_best_idx(1), 1, 1);
    best_Ki = pop(global_best_idx(1), 1, 2);
    best_Kd = pop(global_best_idx(1), 1, 3);
    
    fprintf('Gen %2d | Global Best Score: %6.2f | Cost: %8.2f | Drops: %2d | Elite -> Kp: %4.2f, Ki: %4.2f, Kd: %4.2f\n', ...
        gen, global_best_rob, global_best_fit, global_best_drops, best_Kp, best_Ki, best_Kd);
        
    if show_plot && ishandle(fig)
        set(h_best, 'XData', 1:gen, 'YData', best_score_history(1:gen));
        xlim(ax, [1 generations]);
        ylim(ax, [0 max(best_score_history(1:gen)) * 1.2]);
        drawnow;
    end
    
    if gen > 35
        if (best_fitness_history(gen-35) - global_best_fit) > -0.001 && (best_fitness_history(gen-35) - global_best_fit) < 0.001
            fprintf('Early stopping triggered at generation %d.\n', gen);
            break;
        end
    end
end

fprintf('\n================================================\n');
fprintf('ISLAND GA EVOLUTION COMPLETE!\n');
fprintf('Optimal PID Parameters Found:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.3f\n', best_Ki);
fprintf('  Kd = %.3f\n', best_Kd);
fprintf('================================================\n');

end
