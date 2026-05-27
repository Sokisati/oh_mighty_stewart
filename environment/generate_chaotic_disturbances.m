function F_table = generate_chaotic_disturbances(seed, T_sim, params)

    if nargin < 3 || isempty(params)
        config = load_config();
        params.min_interval = config.chao_min_interval;
        params.max_interval = config.chao_max_interval;
        params.min_force    = config.chao_min_force;
        params.max_force    = config.chao_max_force;
        params.angle_drift_speed = config.chao_angle_drift_speed;
        params.angle_spread = config.chao_angle_spread;
        params.anti_cancel_deg = config.chao_anti_cancel_deg;
    end

    rng(seed);
    
    t_current = 0;
    F_table = [];
    prev_angle = NaN;
    
    while true
        dt_next = params.min_interval + rand() * (params.max_interval - params.min_interval);
        t_current = t_current + dt_next;
        
        if t_current >= T_sim
            break;
        end
        
        dom_angle = t_current * params.angle_drift_speed;
        
        angle_deg = dom_angle + randn() * params.angle_spread;
        
        if ~isnan(prev_angle)
            diff_mod = mod(angle_deg - prev_angle, 360);
            
            if abs(diff_mod - 180) < params.anti_cancel_deg
                if diff_mod > 180
                    angle_deg = angle_deg + params.anti_cancel_deg;
                else
                    angle_deg = angle_deg - params.anti_cancel_deg;
                end
            end
        end
        prev_angle = mod(angle_deg, 360);
        
        force = params.min_force + rand() * (params.max_force - params.min_force);
        
        angle_rad = angle_deg * pi / 180;
        vx = force * cos(angle_rad);
        vy = force * sin(angle_rad);
        
        F_table = [F_table; t_current, vx, vy];
    end
end
