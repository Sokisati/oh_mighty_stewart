function F_table = generate_realistic_disturbances(seed, T_sim, params)

    if nargin < 3 || isempty(params)
        config = load_config();
        params.min_interval = config.real_min_interval;
        params.max_interval = config.real_max_interval;
        params.min_force    = config.real_min_force;
        params.max_force    = config.real_max_force;
        params.angle_spread = config.real_angle_spread;
    end

    rng(seed);
    
    base_angle_deg = rand() * 360; 
    
    t_current = 0;
    F_table = [];
    
    while true
        dt_next = params.min_interval + rand() * (params.max_interval - params.min_interval);
        t_current = t_current + dt_next;
        
        if t_current >= T_sim
            break;
        end
        
        angle_variation = randn() * params.angle_spread;
        angle_variation = max(-30, min(30, angle_variation));
        
        angle_deg = base_angle_deg + angle_variation;
        
        force = params.min_force + rand() * (params.max_force - params.min_force);
        
        angle_rad = angle_deg * pi / 180;
        vx = force * cos(angle_rad);
        vy = force * sin(angle_rad);
        
        F_table = [F_table; t_current, vx, vy];
    end
end
