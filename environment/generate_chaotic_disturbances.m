function F_table = generate_chaotic_disturbances(seed, T_sim, params)
% GENERATE_CHAOTIC_DISTURBANCES Produces a sequence of pseudo-random disturbance kicks.
%
%   F_table = generate_chaotic_disturbances(seed, T_sim, params)
%
%   Inputs:
%       seed   : Random seed for reproducibility (e.g., 1, 2, 3...)
%       T_sim  : Total simulation time [s]
%       params : (Optional) Struct with tuning parameters
%
%   Output:
%       F_table: [N x 3] matrix where each row is:
%                [time(s), kick_vx(m/s), kick_vy(m/s)]

    if nargin < 3 || isempty(params)
        params.min_interval = 1.8;   % [s]
        params.max_interval = 3;   % [s]
        params.min_force    = 0.45;  % [m/s]
        params.max_force    = 0.75;  % [m/s]
        params.angle_drift_speed = 25; % [deg/s] - how fast the dominant angle rotates
        params.angle_spread = 30;      % [deg]   - std dev of random scatter around dominant angle
        params.anti_cancel_deg = 45;   % [deg]   - minimum angle difference from exactly opposite
    end

    % Initialize deterministic RNG
    rng(seed);
    
    t_current = 0;
    F_table = [];
    prev_angle = NaN;
    
    while true
        % 1. Determine time of next kick
        dt_next = params.min_interval + rand() * (params.max_interval - params.min_interval);
        t_current = t_current + dt_next;
        
        if t_current >= T_sim
            break;
        end
        
        % 2. Determine base angle (drifting over time)
        dom_angle = t_current * params.angle_drift_speed;
        
        % 3. Add random spread to base angle
        angle_deg = dom_angle + randn() * params.angle_spread;
        
        % 4. Anti-Cancellation Filter
        if ~isnan(prev_angle)
            % Difference between current and previous angle
            diff_mod = mod(angle_deg - prev_angle, 360);
            
            % If the difference is too close to 180 degrees (direct opposite)
            if abs(diff_mod - 180) < params.anti_cancel_deg
                if diff_mod > 180
                    angle_deg = angle_deg + params.anti_cancel_deg;
                else
                    angle_deg = angle_deg - params.anti_cancel_deg;
                end
            end
        end
        prev_angle = mod(angle_deg, 360);
        
        % 5. Determine force magnitude
        force = params.min_force + rand() * (params.max_force - params.min_force);
        
        % 6. Convert to X-Y components
        angle_rad = angle_deg * pi / 180;
        vx = force * cos(angle_rad);
        vy = force * sin(angle_rad);
        
        % Append to table
        F_table = [F_table; t_current, vx, vy];
    end
end
