function F_table = generate_realistic_disturbances(seed, T_sim, params)
% GENERATE_REALISTIC_DISTURBANCES Produces a sequence of wind disturbance kicks
% blowing from a single consistent direction, similar to real weather.
%
%   F_table = generate_realistic_disturbances(seed, T_sim, params)
%
%   Inputs:
%       seed   : Random seed for reproducibility
%       T_sim  : Total simulation time [s]
%       params : (Optional) Struct with tuning parameters
%
%   Output:
%       F_table: [N x 3] matrix where each row is:
%                [time(s), kick_vx(m/s), kick_vy(m/s)]

    if nargin < 3 || isempty(params)
        params.min_interval = 0.3;   % [s] (slightly more frequent than chaotic)
        params.max_interval = 0.4;   % [s]
        params.min_force    = 0.1;  % [m/s]
        params.max_force    = 0.2;  % [m/s]
        params.angle_spread = 10;    % [deg] - std dev of random scatter around base angle
    end

    % Initialize deterministic RNG
    rng(seed);
    
    % Determine the primary blowing direction for this entire simulation
    % (e.g. "Wind is blowing from the Northeast today")
    base_angle_deg = rand() * 360; 
    
    t_current = 0;
    F_table = [];
    
    while true
        % 1. Determine time of next kick
        dt_next = params.min_interval + rand() * (params.max_interval - params.min_interval);
        t_current = t_current + dt_next;
        
        if t_current >= T_sim
            break;
        end
        
        % 2. Add random spread to the FIXED base angle
        % Using randn() gives a normal distribution around the base_angle
        % We clip it to +/- 30 degrees to avoid physically impossible sudden U-turns
        angle_variation = randn() * params.angle_spread;
        angle_variation = max(-30, min(30, angle_variation));
        
        angle_deg = base_angle_deg + angle_variation;
        
        % 3. Determine force magnitude
        force = params.min_force + rand() * (params.max_force - params.min_force);
        
        % 4. Convert to X-Y components
        angle_rad = angle_deg * pi / 180;
        vx = force * cos(angle_rad);
        vy = force * sin(angle_rad);
        
        % Append to table
        F_table = [F_table; t_current, vx, vy];
    end
end
