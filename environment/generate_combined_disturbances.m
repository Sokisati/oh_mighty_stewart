function F_table = generate_combined_disturbances(seed, c_ratio, r_ratio, T_sim)
% GENERATE_COMBINED_DISTURBANCES Produces a combined sequence of chaotic and realistic wind kicks.
%
%   F_table = generate_combined_disturbances(seed, c_ratio, r_ratio, T_sim)
%
%   Inputs:
%       seed    : Random seed for reproducibility
%       c_ratio : Coefficient for chaotic wind (0 to 1). Affects amplitude and frequency.
%       r_ratio : Coefficient for realistic wind (0 to 1). Affects amplitude and frequency.
%       T_sim   : Total simulation time [s]
%
%   Output:
%       F_table : [N x 3] matrix sorted by time:
%                 [time(s), kick_vx(m/s), kick_vy(m/s)]

    if nargin < 4
        T_sim = 30;
    end
    
    % Generate full base tables using the same seed.
    % They won't sync up because their interval parameters and rand() call counts differ.
    F_c = generate_chaotic_disturbances(seed, T_sim);
    F_r = generate_realistic_disturbances(seed, T_sim);
    
    rng(seed + 2000); % RNG for probability filtering
    
    F_combined = [];
    
    % Filter and scale Chaotic events
    for i = 1:size(F_c, 1)
        if rand() <= c_ratio
            F_combined = [F_combined; F_c(i, 1), F_c(i, 2)*c_ratio, F_c(i, 3)*c_ratio];
        end
    end
    
    % Filter and scale Realistic events
    for i = 1:size(F_r, 1)
        if rand() <= r_ratio
            F_combined = [F_combined; F_r(i, 1), F_r(i, 2)*r_ratio, F_r(i, 3)*r_ratio];
        end
    end
    
    % Sort chronological by time
    if ~isempty(F_combined)
        F_table = sortrows(F_combined, 1);
    else
        F_table = zeros(0, 3);
    end
end
