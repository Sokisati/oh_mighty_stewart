function [global_best_Kp, global_best_Ki, global_best_Kd] = stewart_cmaes_multi_seed(num_scenarios, base_lambda_pop, max_generations, show_plot, rng_seed)

config = load_config();
if nargin < 1, num_scenarios = 100; end
if nargin < 2, base_lambda_pop = config.cmaes_lambda; end
if nargin < 3, max_generations = max(100, config.cmaes_generations); end % Give CMA-ES more time for restarts
if nargin < 4, show_plot = true; end
if nargin < 5, rng_seed = 42; end

deg2rad = pi / 180;

clc;
if show_plot, close all; end

fprintf('\n================================================\n');
fprintf('  ROBUST IPOP-CMA-ES (Increasing Pop Restart CMA-ES)\n');
fprintf('================================================\n');

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

UB = [25.0, 40.0, 10.0];
LB = [0.0,  0.0,  0.0];
N = 3; 

global_best_fit = inf;
global_best_Kp = 0; global_best_Ki = 0; global_best_Kd = 0;
global_best_rob = 0;

max_tilt_rad = 30 * deg2rad;

lambda_pop = base_lambda_pop;
gen = 1;
restarts = 0;

if show_plot
    fig = figure('Name', 'Robust IPOP-CMA-ES Evolution', 'Color', [0.1 0.1 0.12], 'Position', [200 200 800 500]);
    ax = axes('Parent', fig, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w');
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, sprintf('IPOP-CMA-ES Evolution (Best Score across %d Scenarios)', num_scenarios), 'Color', 'w', 'FontSize', 12);
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

best_score_history = nan(max_generations, 1);

while gen <= max_generations
    fprintf('\n--- RESTART %d (Pop Size: %d) ---\n', restarts, lambda_pop);
    
    xmean = LB' + rand(N,1) .* (UB' - LB'); 
    sigma = 12.0; % Increased significantly for aggressive initial exploration

    mu = floor(lambda_pop / 2);
    weights = log(mu + 1/2) - log(1:mu)'; 
    weights = weights / sum(weights);
    mueff = sum(weights)^2 / sum(weights.^2);

    cc = (4 + mueff/N) / (N+4 + 2*mueff/N);
    cs = (mueff+2) / (N+mueff+5);
    c1 = 2 / ((N+1.3)^2 + mueff);
    cmu = min(1-c1, 2 * (mueff-2+1/mueff) / ((N+2)^2 + mueff));
    damps = 1 + 2*max(0, sqrt((mueff-1)/(N+1))-1) + cs;

    pc = zeros(N, 1);
    ps = zeros(N, 1);
    B = eye(N, N);
    D = ones(N, 1);
    C = B * diag(D.^2) * B';
    invsqrtC = B * diag(D.^-1) * B';
    eigeneval = 0;
    chiN = N^0.5 * (1 - 1/(4*N) + 1/(21*N^2));
    
    stagnation_counter = 0;
    last_best_fit = inf;

    while gen <= max_generations
        pop = zeros(N, lambda_pop);
        for k = 1:lambda_pop
            pop(:, k) = xmean + sigma * B * (D .* randn(N,1));
            for g = 1:N
                if pop(g,k) > UB(g)
                    pop(g,k) = UB(g) - (pop(g,k) - UB(g));
                elseif pop(g,k) < LB(g)
                    pop(g,k) = LB(g) + (LB(g) - pop(g,k));
                end
                pop(g,k) = max(LB(g), min(UB(g), pop(g,k)));
            end
        end

        fitness_scores = zeros(lambda_pop, 1);
        robust_scores = zeros(lambda_pop, 1);
        num_drops = zeros(lambda_pop, 1);

        parfor k = 1:lambda_pop
            [fitness_scores(k), robust_scores(k), num_drops(k)] = evaluate_fitness_robust(pop(:, k)', h0_val, r_limit_val, g_acc_val, c_roll_val, max_tilt_rad, disturbances, noises);
        end
        
        [fitness_scores, sort_idx] = sort(fitness_scores);
        pop = pop(:, sort_idx);
        robust_scores = robust_scores(sort_idx);
        num_drops = num_drops(sort_idx);
        
        best_fit = fitness_scores(1);
        best_rob = robust_scores(1);
        best_drops  = num_drops(1);
        
        if best_fit < global_best_fit
            global_best_fit = best_fit;
            global_best_rob = best_rob;
            global_best_Kp = pop(1,1);
            global_best_Ki = pop(2,1);
            global_best_Kd = pop(3,1);
        end
        
        best_score_history(gen) = global_best_rob;
        
        fprintf('Gen %3d | Global Best Score: %6.2f | Cost: %8.2f | Drops: %2d | Local Elite -> Kp: %4.2f, Ki: %4.2f, Kd: %4.2f\n', ...
            gen, global_best_rob, best_fit, best_drops, pop(1,1), pop(2,1), pop(3,1));
            
        if show_plot && ishandle(fig)
            valid_idx = find(~isnan(best_score_history));
            set(h_best, 'XData', valid_idx, 'YData', best_score_history(valid_idx));
            xlim(ax, [1 max_generations]);
            ylim(ax, [0 max(best_score_history(valid_idx)) * 1.2]);
            drawnow;
        end
        
        gen = gen + 1;
        
        if abs(last_best_fit - best_fit) < 1e-3
            stagnation_counter = stagnation_counter + 1;
        else
            stagnation_counter = 0;
            last_best_fit = best_fit;
        end
        
        if stagnation_counter >= 25 || sigma < 1e-4 || cond(C) > 1e14
            fprintf('  [IPOP Triggered] Local minimum detected (Stagnation or flat variance). Restarting...\n');
            lambda_pop = lambda_pop * 2; % Double population
            restarts = restarts + 1;
            break; % Break inner loop to trigger restart
        end
        
        xold = xmean;
        xmean = pop(:, 1:mu) * weights;
        
        ps = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC * (xmean-xold) / sigma;
        hsig = norm(ps)/sqrt(1-(1-cs)^(2*gen))/chiN < 1.4 + 2/(N+1);
        pc = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * (xmean-xold) / sigma;
        
        artmp = (1/sigma) * (pop(:, 1:mu) - repmat(xold, 1, mu));
        C = (1-c1-cmu) * C ...
            + c1 * (pc*pc' + (1-hsig) * cc*(2-cc) * C) ...
            + cmu * artmp * diag(weights) * artmp';
            
        sigma = sigma * exp((cs/damps)*(norm(ps)/chiN - 1));
        
        if gen - eigeneval > lambda_pop/(c1+cmu)/N/10
            eigeneval = gen;
            C = triu(C) + triu(C,1)';
            [B, D2] = eig(C);
            D = sqrt(diag(D2));
            invsqrtC = B * diag(D.^-1) * B';
        end
    end
end

fprintf('\n================================================\n');
fprintf('IPOP-CMA-ES EVOLUTION COMPLETE (Total Restarts: %d)!\n', restarts);
fprintf('Optimal PID Parameters Found:\n');
fprintf('  Kp = %.3f\n', global_best_Kp);
fprintf('  Ki = %.3f\n', global_best_Ki);
fprintf('  Kd = %.3f\n', global_best_Kd);
fprintf('================================================\n');

end
