
function out = load_config()
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
    
    if nargout > 0
        out = config_struct;
    else
        fields = fieldnames(config_struct);
        for f = 1:length(fields)
            assignin('caller', fields{f}, config_struct.(fields{f}));
        end
    end
end
