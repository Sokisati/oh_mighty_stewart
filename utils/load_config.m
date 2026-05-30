function out = load_config()
% LOAD_CONFIG Loads parameters from config.txt and returns them as a struct.
% Dynamically calculates Kp, Ki, Kd based on delay_sec, slew_rate_deg, 
% and c_roll. Overwrites config.txt on disk if parameters have changed.

    utils_dir = fileparts(mfilename('fullpath'));
    proj_dir = fileparts(utils_dir);
    config_path = fullfile(proj_dir, 'config.txt');
    
    if ~exist(config_path, 'file')
        error('Configuration file not found at: %s', config_path);
    end
    
    fid = fopen(config_path, 'r');
    if fid == -1
        error('Failed to open configuration file: %s', config_path);
    end
    
    config_struct = struct();
    
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line), break; end
        
        line = strtrim(line);
        
        if isempty(line) || startsWith(line, '#') || startsWith(line, '%')
            continue;
        end
        
        eq_idx = strfind(line, '=');
        if isempty(eq_idx), continue; end
        
        key = strtrim(line(1:eq_idx(1)-1));
        val_str = line(eq_idx(1)+1:end);
        
        comment_idx = [strfind(val_str, '#'), strfind(val_str, '%')];
        if ~isempty(comment_idx)
            val_str = val_str(1:min(comment_idx)-1);
        end
        val_str = strtrim(val_str);
        
        val = str2double(val_str);
        if isnan(val)
            val = val_str;
        end
        
        config_struct.(key) = val;
    end
    
    fclose(fid);
    
    % --- DYNAMIC PARAMETRIC GAIN OVERWRITE & DISK UPDATE ---
    try
        g_acc = config_struct.g_acc;
        delay_sec = config_struct.delay_sec;
        slew_rate_deg = config_struct.slew_rate_deg;
        c_roll = config_struct.c_roll;
        
        % 1. Dynamic Natural Frequency based on delay
        omega_n = 0.303 / max(0.005, delay_sec);
        omega_n = min(20.0, max(3.0, omega_n));
        
        % 2. Dynamic Damping Ratio based on slew rate limit
        zeta = 0.8 * max(1.0, 150 / slew_rate_deg);
        zeta = min(1.5, max(0.7, zeta));
        
        % 3. Parametric Proportional & Derivative gains (considering rolling damping c_roll)
        Kp_new = (7/5) * (omega_n^2 / g_acc);
        Kd_new = max(0.0, (7/5) * (2 * zeta * omega_n - c_roll) / g_acc);
        
        % 4. Parametric Integrator placing (T_i = 12.38 * T_n)
        T_n = 2 * pi / omega_n;
        T_i = 12.38 * T_n;
        Ki_new = Kp_new / T_i;
        
        % Check if the values on disk need to be updated
        needs_update = false;
        if ~isfield(config_struct, 'Kp') || abs(config_struct.Kp - Kp_new) > 1e-4 || ...
           ~isfield(config_struct, 'Ki') || abs(config_struct.Ki - Ki_new) > 1e-4 || ...
           ~isfield(config_struct, 'Kd') || abs(config_struct.Kd - Kd_new) > 1e-4
            needs_update = true;
        end
        
        if needs_update
            fprintf('Updating config.txt with new parametric analytical gains...\n');
            fprintf('  Old -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', config_struct.Kp, config_struct.Ki, config_struct.Kd);
            fprintf('  New -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_new, Ki_new, Kd_new);
            
            % Read original config file lines
            fid_r = fopen(config_path, 'r');
            lines = {};
            while ~feof(fid_r)
                lines{end+1} = fgetl(fid_r);
            end
            fclose(fid_r);
            
            % Modify Kp, Ki, Kd lines
            for l = 1:length(lines)
                if ~ischar(lines{l}), continue; end
                line_trim = strtrim(lines{l});
                if startsWith(line_trim, 'Kp') && contains(line_trim, '=')
                    lines{l} = sprintf('Kp = %.4f', Kp_new);
                elseif startsWith(line_trim, 'Ki') && contains(line_trim, '=')
                    lines{l} = sprintf('Ki = %.4f', Ki_new);
                elseif startsWith(line_trim, 'Kd') && contains(line_trim, '=')
                    lines{l} = sprintf('Kd = %.4f', Kd_new);
                end
            end
            
            % Write back to config.txt
            fid_w = fopen(config_path, 'w');
            for l = 1:length(lines)
                fprintf(fid_w, '%s\n', lines{l});
            end
            fclose(fid_w);
            
            % Update returned config struct fields
            config_struct.Kp = Kp_new;
            config_struct.Ki = Ki_new;
            config_struct.Kd = Kd_new;
        end
    catch ME
        fprintf('Warning: Failed to auto-update analytical gains in config.txt: %s\n', ME.message);
    end
    % -------------------------------------------------------
    
    if nargout > 0
        out = config_struct;
    else
        fields = fieldnames(config_struct);
        for f = 1:length(fields)
            assignin('caller', fields{f}, config_struct.(fields{f}));
        end
    end
end
