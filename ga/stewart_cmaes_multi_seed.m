function [best_Kp, best_Ki, best_Kd] = stewart_cmaes_multi_seed(num_scenarios, lambda_pop, generations, show_plot, rng_seed)
%% stewart_cmaes_multi_seed.m
%  Robust Covariance Matrix Adaptation Evolution Strategy (CMA-ES)

config = load_config();
if nargin < 1, num_scenarios = 100; end
if nargin < 2, lambda_pop = config.cmaes_lambda; end
if nargin < 3, generations = config.cmaes_generations; end
if nargin < 4, show_plot = true; end
if nargin < 5, rng_seed = 42; end

deg2rad = pi / 180;

clc;
if show_plot, close all; end

fprintf('\n================================================\n');
fprintf('  ROBUST CMA-ES (Covariance Matrix Adaptation)\n');
fprintf('================================================\n');
fprintf('Training config: Scenarios = %d, Pop (lambda) = %d, Generations = %d\n', num_scenarios, lambda_pop, generations);

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

% Bounds
UB = [25.0, 40.0, 10.0];
LB = [0.0,  0.0,  0.0];

N = 3; % Dimension
xmean = LB' + rand(N,1) .* (UB' - LB'); % Initial mean
sigma = 5.0; % Coordinate wise initial step size

mu = floor(lambda_pop / 2);
weights = log(mu + 1/2) - log(1:mu)'; % muXone array for weights
weights = weights / sum(weights);
mueff = sum(weights)^2 / sum(weights.^2);

% Strategy parameter setting: Adaptation
cc = (4 + mueff/N) / (N+4 + 2*mueff/N);
cs = (mueff+2) / (N+mueff+5);
c1 = 2 / ((N+1.3)^2 + mueff);
cmu = min(1-c1, 2 * (mueff-2+1/mueff) / ((N+2)^2 + mueff));
damps = 1 + 2*max(0, sqrt((mueff-1)/(N+1))-1) + cs;

% Initialize dynamic (internal) strategy parameters and constants
pc = zeros(N, 1);
ps = zeros(N, 1);
B = eye(N, N);
D = ones(N, 1);
C = B * diag(D.^2) * B';
invsqrtC = B * diag(D.^-1) * B';
eigeneval = 0;
chiN = N^0.5 * (1 - 1/(4*N) + 1/(21*N^2));

FAILURE_THRESHOLD = 5000;

best_fitness_history = zeros(generations, 1);
avg_fitness_history = zeros(generations, 1);
best_score_history = zeros(generations, 1);
avg_score_history = zeros(generations, 1);

if show_plot
    fig = figure('Name', 'Robust CMA-ES Evolution', 'Color', [0.1 0.1 0.12], 'Position', [200 200 800 500]);
    ax = axes('Parent', fig, 'Color', [0.15 0.15 0.18], 'XColor', 'w', 'YColor', 'w');
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, sprintf('Robust CMA-ES Evolution (Best Score across %d Scenarios)', num_scenarios), 'Color', 'w', 'FontSize', 12);
    xlabel(ax, 'Generation', 'Color', 'w');
    ylabel(ax, 'Average Benchmark Score', 'Color', 'w');
    h_best = plot(ax, NaN, NaN, 'g.-', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', 'Best Fitness');
    h_avg  = plot(ax, NaN, NaN, 'y.--', 'LineWidth', 1, 'MarkerSize', 10, 'DisplayName', 'Population Average');
    legend(ax, 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
end

max_tilt_rad = 30 * deg2rad;

for gen = 1:generations
    rng(rng_seed + gen * 100);
    seeds = randi([1, 100000], 1, num_scenarios);
    disturbances = cell(num_scenarios, 1);
    noises = cell(num_scenarios, 1);
    for s = 1:num_scenarios
        disturbances{s} = generate_disturbances(seeds(s), 30.0);
        noises{s} = generate_sensor_noise(seeds(s), 30.0, 0.02);
    end

    pop = zeros(N, lambda_pop);
    for k = 1:lambda_pop
        pop(:, k) = xmean + sigma * B * (D .* randn(N,1));
        % Reflective bounds
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
        gen, best_robust, best_drops, pop(1,1), pop(2,1), pop(3,1));
        
    if show_plot && ishandle(fig)
        set(h_best, 'XData', 1:gen, 'YData', best_score_history(1:gen));
        set(h_avg,  'XData', 1:gen, 'YData', avg_score_history(1:gen));
        xlim(ax, [1 generations]);
        ylim(ax, [0 max(best_score_history(1:gen)) * 1.2]);
        drawnow;
    end
    
    if gen > 20
        if (best_fitness_history(gen-20) - best_fitness) < 0.001
            fprintf('Early stopping triggered at generation %d.\n', gen);
            break;
        end
    end
    
    xold = xmean;
    xmean = pop(:, 1:mu) * weights;
    
    % Cumulation
    ps = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC * (xmean-xold) / sigma;
    hsig = norm(ps)/sqrt(1-(1-cs)^(2*gen))/chiN < 1.4 + 2/(N+1);
    pc = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * (xmean-xold) / sigma;
    
    % Adapt covariance matrix C
    artmp = (1/sigma) * (pop(:, 1:mu) - repmat(xold, 1, mu));
    C = (1-c1-cmu) * C ...
        + c1 * (pc*pc' + (1-hsig) * cc*(2-cc) * C) ...
        + cmu * artmp * diag(weights) * artmp';
        
    % Adapt step size sigma
    sigma = sigma * exp((cs/damps)*(norm(ps)/chiN - 1));
    
    % Update B and D from C
    if gen - eigeneval > lambda_pop/(c1+cmu)/N/10
        eigeneval = gen;
        C = triu(C) + triu(C,1)';
        [B, D2] = eig(C);
        D = sqrt(diag(D2));
        invsqrtC = B * diag(D.^-1) * B';
    end
end

best_Kp = pop(1,1);
best_Ki = pop(2,1);
best_Kd = pop(3,1);

fprintf('\n================================================\n');
fprintf('ROBUST CMA-ES EVOLUTION COMPLETE!\n');
fprintf('Optimal PID Parameters Found:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.3f\n', best_Ki);
fprintf('  Kd = %.3f\n', best_Kd);
fprintf('================================================\n');

end
