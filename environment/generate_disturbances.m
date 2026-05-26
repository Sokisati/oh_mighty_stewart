function F_table = generate_disturbances(seed, T_sim, params)
% GENERATE_DISTURBANCES Wrapper function to select between Chaotic or Realistic wind.
% The wind type is determined by the global variable WIND_TYPE.

    global WIND_TYPE;
    global WIND_C_RATIO;
    global WIND_R_RATIO;
    
    if isempty(WIND_TYPE)
        WIND_TYPE = 'chaotic'; % Default fallback
    end
    
    if strcmpi(WIND_TYPE, 'realistic')
        if nargin < 3
            F_table = generate_realistic_disturbances(seed, T_sim);
        else
            F_table = generate_realistic_disturbances(seed, T_sim, params);
        end
    elseif strcmpi(WIND_TYPE, 'combined')
        if isempty(WIND_C_RATIO), WIND_C_RATIO = 0.3; end
        if isempty(WIND_R_RATIO), WIND_R_RATIO = 1.0; end
        F_table = generate_combined_disturbances(seed, WIND_C_RATIO, WIND_R_RATIO, T_sim);
    else
        if nargin < 3
            F_table = generate_chaotic_disturbances(seed, T_sim);
        else
            F_table = generate_chaotic_disturbances(seed, T_sim, params);
        end
    end
end
