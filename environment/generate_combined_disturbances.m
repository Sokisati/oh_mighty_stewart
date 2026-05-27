function F_table = generate_combined_disturbances(seed, c_ratio, r_ratio, T_sim)

    if nargin < 4
        T_sim = 30;
    end
    
    F_c = generate_chaotic_disturbances(seed, T_sim);
    F_r = generate_realistic_disturbances(seed, T_sim);
    
    rng(seed + 2000); % RNG for probability filtering
    
    F_combined = [];
    
    for i = 1:size(F_c, 1)
        if rand() <= c_ratio
            F_combined = [F_combined; F_c(i, 1), F_c(i, 2)*c_ratio, F_c(i, 3)*c_ratio];
        end
    end
    
    for i = 1:size(F_r, 1)
        if rand() <= r_ratio
            F_combined = [F_combined; F_r(i, 1), F_r(i, 2)*r_ratio, F_r(i, 3)*r_ratio];
        end
    end
    
    if ~isempty(F_combined)
        F_table = sortrows(F_combined, 1);
    else
        F_table = zeros(0, 3);
    end
end
