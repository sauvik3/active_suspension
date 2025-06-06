clc;
clear;
close all;

%% ----------------- Setup Path -----------------
base_path = get_base_path();
disp(['Base Path: ', base_path]);

sub_dirs = {'algo', 'controller', 'init', 'utils'};
for i = 1:numel(sub_dirs)
    full_path = fullfile(base_path, sub_dirs{i});
    if exist(full_path, 'dir')
        addpath(full_path);
    else
        warning('Missing path: %s', full_path);
    end
end

% Run CRG init if available
crg_init_script = fullfile(base_path, 'crg_dataset', 'ASAM_OpenCRG_BS_V1.2.0', 'matlab', 'crg_init.m');
if exist(crg_init_script, 'file')
    run(crg_init_script);
else
    warning('CRG init script not found: %s', crg_init_script);
end

% Load system parameters
run(fullfile(base_path, 'init', 'parameters.m'));

%% ----------------- Controller Setup -----------------
controllers = {'PID', 'SMC', 'LQR', 'Hinf'};
gain_map = gains_lookup().CRG;

%% ----------------- Speed Sweep Parameters -----------------
speeds_kmh = [10, 30, 50, 70, 90]; % Speeds in km/h
speeds_mps = speeds_kmh / 3.6;     % Convert to m/s

%% ----------------- Simulation Loop (Speed Sweep) -----------------

% Get CRG profiles
crg_profiles = fieldnames(gain_map.(controllers{1}));

% Loop through each CRG class
for i = 1:numel(crg_profiles)
    crg_name = crg_profiles{i};

    %% ----------------- Process CRG File -----------------
    crg_folder = fullfile(base_path, 'crg_dataset/ASAM_OpenCRG_BS_V1.2.0/crg-bin');
    crg_file = fullfile(crg_folder, 'country_road.crg');

    % Extract filename with extension
    [~, crg_fname, crg_ext] = fileparts(crg_file);
    fprintf('\nProcessing CRG File: %s%s\n', crg_fname, crg_ext);

    % Calculate T_plot once, using the slowest speed to define the base road profile
    T_plot = 0:resolution:(road_len / speeds_mps(1));

    [road_length, ~, road_profile] = process_crg_file(crg_file, T_plot, speeds_mps(1));

    fprintf('\n===== CRG Profile: %s - Speed Sweep =====\n', crg_name);

    % Loop through each controller
    for ctrl_idx = 1:numel(controllers)
        controller = controllers{ctrl_idx};
        controller_func = str2func(['run_', lower(controller), '_control']);

        if ~isfield(gain_map, controller) || ~isfield(gain_map.(controller), crg_name)
            warning('Gains for %s in CRG-%s not found. Skipping.', controller, crg_name);
            continue;
        end

        % State Space model
        [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);
        gains = gain_map.(controller).(crg_name);

        fprintf('\n----- Controller: %s -----\n', controller);

        all_speed_results = struct();

        for s_idx = 1:numel(speeds_mps)
            current_speed = speeds_mps(s_idx);
            fprintf('\n--- Speed: %.1f km/h (%.2f m/s) ---\n', speeds_kmh(s_idx), current_speed);

            simulation_time = road_len / current_speed;
            T_speed = 0:resolution:simulation_time;

            % Resample road profile for current speed
            % Use the 'road_profile' (base profile) and 'T_plot' (base time) for interpolation
            road_profile_current_speed = interp1(T_plot * speeds_mps(1), road_profile, T_speed * current_speed);

            label = sprintf('%s - %s - %.1f km/h', controller, crg_name, speeds_kmh(s_idx));
            key = matlab.lang.makeValidName(sprintf('%s_%s_%dkmh', controller, crg_name, speeds_kmh(s_idx)));

            try
                [t, y] = controller_func(A, B, T_speed, road_profile_current_speed, x0, gains);
                all_speed_results.(key).t = t;
                all_speed_results.(key).y = y;
                all_speed_results.(key).label = label;
                all_speed_results.(key).speed = current_speed;
            catch err
                warning(err.identifier, 'Simulation failed for %s at %.1f km/h: %s', crg_name, speeds_kmh(s_idx), err.message);
                all_speed_results.(key).t = T_speed;
                all_speed_results.(key).y = NaN(numel(T_speed), size(B, 2));
                all_speed_results.(key).label = [label, ' (error)'];
                all_speed_results.(key).speed = current_speed;
            end
        end

        title_str = sprintf('Speed Sweep - %s - %s', controller, crg_name);
        display_metrics_table(title_str, all_speed_results);

        % Use the base T_plot and road_profile (the one calculated once for the slowest speed) for plotting
        if exist('T_plot', 'var') && exist('road_profile', 'var')
            plot_results(T_plot * speeds_mps(1), road_profile, title_str, crg_name, all_speed_results, plot_flag, crg_name);
        else
            warning('T_plot or road_profile not defined for plotting.');
        end
        fprintf('------------------------------\n');
    end
end

%% ----------------- Local Functions -----------------
function base_path = get_base_path()
    if endsWith(pwd, 'simulation')
        base_path = pwd;
    else
        base_path = fullfile(pwd, 'simulation');
    end
end
