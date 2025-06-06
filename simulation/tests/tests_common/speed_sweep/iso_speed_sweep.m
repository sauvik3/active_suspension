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

% Load system parameters
run(fullfile(base_path, 'init', 'parameters.m'));

%% ----------------- Controller Setup -----------------
controllers = {'PID', 'SMC', 'LQR', 'Hinf'};
gain_map = gains_lookup().ISO;

%% ----------------- Speed Sweep Parameters -----------------
speeds_kmh = [10, 30, 50, 70, 90]; % Speeds in km/h
speeds_mps = speeds_kmh / 3.6;     % Convert to m/s

%% ----------------- Simulation Loop (Speed Sweep) -----------------

% Get ISO classes
iso_classes = fieldnames(gain_map.(controllers{1}));

% Loop through each ISO class
for i = 1:numel(iso_classes)
    iso_class = iso_classes{i};

    % Retrieve road profile parameters (dependent on ISO class)
    % Calculate T_plot once per ISO class, using the slowest speed for the base road profile
    T_plot = 0:resolution:(road_len / speeds_mps(1));
    class_psd = class_psd_values(i);
    u_plot = generate_road_profile(road_len, resolution, T_plot, class_psd);

    % State-Space Model
    [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);

    fprintf('\n===== ISO Profile: %s - Speed Sweep =====\n', iso_class);

    % Loop through each controller
    for ctrl_idx = 1:numel(controllers)
        controller = controllers{ctrl_idx};
        controller_func = str2func(['run_', lower(controller), '_control']);

        % Check if gains exist for the current controller and ISO class
        if ~isfield(gain_map, controller) || ~isfield(gain_map.(controller), iso_class)
            warning('Gains for %s in ISO-%s not found. Skipping.', controller, iso_class);
            continue;
        end

        % Get the base gains for the current controller and ISO class
        gains = gain_map.(controller).(iso_class);

        fprintf('\n----- Controller: %s -----\n', controller);

        all_speed_results = struct();

        for s_idx = 1:numel(speeds_mps)
            simulation_speed = speeds_mps(s_idx);
            fprintf('\n--- Speed: %.1f km/h (%.2f m/s) ---\n', speeds_kmh(s_idx), simulation_speed);

            simulation_time = road_len / simulation_speed;
            T_speed = 0:resolution:simulation_time;

            % Resample the road profile for the current speed's time vector.
            u_speed = interp1(T_plot, u_plot, T_speed, 'linear', 0); % Use linear interpolation, 0 for out-of-bounds

            label = sprintf('%s - %s - %.1f km/h', controller, iso_class, speeds_kmh(s_idx));
            key = matlab.lang.makeValidName(sprintf('%s_%s_%dkmh', controller, iso_class, speeds_kmh(s_idx)));

            try
                [t, y] = controller_func(A, B, T_speed, u_speed, x0, gains);
                all_speed_results.(key).t = t;
                all_speed_results.(key).y = y;
                all_speed_results.(key).label = label;
                all_speed_results.(key).speed = simulation_speed;
            catch err
                warning(err.identifier, 'Simulation failed for %s at %.1f km/h: %s', iso_class, speeds_kmh(s_idx), err.message);
                all_speed_results.(key).t = T_speed;
                all_speed_results.(key).y = NaN(numel(T_speed), size(B, 2));
                all_speed_results.(key).label = [label, ' (error)'];
                all_speed_results.(key).speed = simulation_speed;
            end
        end

        title_str = sprintf('Speed Sweep - %s - %s', controller, iso_class);
        display_metrics_table(title_str, all_speed_results);

        % Use T_plot and u_plot for the road profile.
        if exist('T_plot', 'var') && exist('u_plot', 'var')
            plot_results(T_plot * speeds_mps(1), u_plot, title_str, iso_class, all_speed_results, plot_flag, iso_class);
        else
            warning('T_plot or u_plot not defined for plotting.');
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
