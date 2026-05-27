function noise_table = generate_sensor_noise(seed, T_sim, dt, params)

    if nargin < 4 || isempty(params)
        config = load_config();
        params.sigma_white = config.noise_sigma_white;
        params.sigma_rw    = config.noise_sigma_rw;
        params.spike_prob  = config.noise_spike_prob;
        params.spike_mag   = config.noise_spike_mag;
    end

    rng(seed);
    
    N = floor(T_sim / dt);
    t_vec = (0:N-1)' * dt;
    
    noise_x = randn(N, 1) * params.sigma_white;
    noise_y = randn(N, 1) * params.sigma_white;
    
    rw_x = cumsum(randn(N, 1) * params.sigma_rw);
    rw_y = cumsum(randn(N, 1) * params.sigma_rw);
    
    noise_x = noise_x + rw_x;
    noise_y = noise_y + rw_y;
    
    spikes_idx = rand(N, 1) < params.spike_prob;
    
    noise_x(spikes_idx) = noise_x(spikes_idx) + (rand(sum(spikes_idx), 1) - 0.5) * 2 * params.spike_mag;
    noise_y(spikes_idx) = noise_y(spikes_idx) + (rand(sum(spikes_idx), 1) - 0.5) * 2 * params.spike_mag;
    
    noise_table = [t_vec, noise_x, noise_y];
end
