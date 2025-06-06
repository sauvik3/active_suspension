%% ----------------- Save Figure Function -----------------
function save_figure(fig_handle, file_name, varargin)
    stack = dbstack;

    % Determine the caller level for path context (stack(3) for main script)
    caller_stack_idx = 2;
    if numel(stack) >= 3
        caller_stack_idx = 3;
    end

    if numel(stack) < caller_stack_idx
        error('Cannot determine caller''s file path at the required stack level.');
    end

    % Get caller's path and resolve its full absolute path
    initial_caller_path = stack(caller_stack_idx).file;
    full_caller_path = which(initial_caller_path);

    if isempty(full_caller_path)
        warning('save_figure:PathResolution', 'Could not resolve full path for caller script "%s".', initial_caller_path);
        resolved_caller_dir = fileparts(initial_caller_path);
        [~, script_name_only, ~] = fileparts(initial_caller_path);
    else
        resolved_caller_dir = fileparts(full_caller_path);
        [~, script_name_only, ~] = fileparts(full_caller_path);
    end

    % Determine the additional path segments for saving
    constructed_base_dir_segments = get_default_figure_path_segments(resolved_caller_dir);
    
    % Start with default segments + script name
    save_dir_segments = [constructed_base_dir_segments, {script_name_only}];

    % If varargin is provided (e.g., from plot_results passing 'controller'), 
    % append its contents as additional directory segments.
    if ~isempty(varargin)
        additional_segments = varargin{1}; 
        save_dir_segments = [save_dir_segments, additional_segments];
    end

    % Construct the full save path
    save_path = fullfile('./figures/fig', save_dir_segments{:});

    % Create directory if it doesn't exist
    if ~isfolder(save_path)
        mkdir(save_path);
    end

    % Construct and save the figure file
    fig_file_path = fullfile(save_path, [file_name, '.fig']);
    try
        saveas(fig_handle, fig_file_path, 'fig');
        fprintf('Figure saved to: %s\n', fig_file_path);
    catch ME
        warning(ME.identifier, 'Failed to save figure: %s', ME.message);
    end
end

% Helper function to derive default figure path segments
function default_segments = get_default_figure_path_segments(resolved_caller_dir)
    path_components = strsplit(resolved_caller_dir, filesep);
    tests_idx = find(strcmp(path_components, 'tests'), 1);

    if ~isempty(tests_idx) % If 'tests' directory is found
        % Directly use components *after* the 'tests' directory
        if numel(path_components) > tests_idx
            constructed_base_dir_segments = path_components(tests_idx + 1:end);
        else
            % If script is directly in 'tests' folder, default to 'tests_root'
            constructed_base_dir_segments = {'tests_root'};
        end
    else
        % 'tests' directory not found, fallback to path relative to CWD
        warning('get_default_figure_path_segments:PathStructure', 'Could not identify "tests" directory. Deriving path relative to CWD.');
        relative_to_cwd = strrep(resolved_caller_dir, pwd, '');

        if startsWith(relative_to_cwd, filesep)
            relative_to_cwd = relative_to_cwd(2:end);
        end

        if isempty(relative_to_cwd)
            constructed_base_dir_segments = {'root_simulation_scripts'};
        else
            constructed_base_dir_segments = strsplit(relative_to_cwd, filesep);
        end
    end

    default_segments = constructed_base_dir_segments;
end
