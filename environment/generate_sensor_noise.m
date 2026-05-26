function noise_table = generate_sensor_noise(seed, T_sim, dt, params)
% GENERATE_SENSOR_NOISE Produces a pseudo-random sequence of camera sensor noise.
%
%   noise_table = generate_sensor_noise(seed, T_sim, dt, params)
%
%   Inputs:
%       seed   : Random seed for reproducibility (e.g., 1, 2, 3...)
%       T_sim  : Total simulation time [s]
%       dt     : Time step of the simulation (e.g., 0.01 for 100 Hz)
%       params : (Optional) Struct with tuning parameters for noise generation
%
%   Output:
%       noise_table: [N x 3] matrix where each row is:
%                    [time(s), noise_x(m), noise_y(m)]

    if nargin < 4 || isempty(params)
        config = load_config();
        params.sigma_white = config.noise_sigma_white;
        params.sigma_rw    = config.noise_sigma_rw;
        params.spike_prob  = config.noise_spike_prob;
        params.spike_mag   = config.noise_spike_mag;
    end

    % Initialize deterministic RNG (Aynı seed = Aynı gürültü profili)
    rng(seed);
    
    N = floor(T_sim / dt);
    t_vec = (0:N-1)' * dt;
    
    % 1. White Noise (Temel kamera piksel gürültüsü)
    noise_x = randn(N, 1) * params.sigma_white;
    noise_y = randn(N, 1) * params.sigma_white;
    
    % 2. Random Walk (Düşük frekanslı sürüklenme/drift - Gölgelenme veya ısınma etkisi)
    % Kameranın merkezi zamanla çok hafifçe kayar.
    rw_x = cumsum(randn(N, 1) * params.sigma_rw);
    rw_y = cumsum(randn(N, 1) * params.sigma_rw);
    
    % White noise ile drift'i birleştir
    noise_x = noise_x + rw_x;
    noise_y = noise_y + rw_y;
    
    % 3. Spikes (Ani parlama, kamera çerçevesi atlama veya anlık okuma hatası)
    spikes_idx = rand(N, 1) < params.spike_prob;
    
    % Eğer spike varsa, mevcut gürültünün üzerine rastgele (negatif veya pozitif) büyük bir hata bindir
    noise_x(spikes_idx) = noise_x(spikes_idx) + (rand(sum(spikes_idx), 1) - 0.5) * 2 * params.spike_mag;
    noise_y(spikes_idx) = noise_y(spikes_idx) + (rand(sum(spikes_idx), 1) - 0.5) * 2 * params.spike_mag;
    
    % Tabloyu oluştur
    noise_table = [t_vec, noise_x, noise_y];
end
