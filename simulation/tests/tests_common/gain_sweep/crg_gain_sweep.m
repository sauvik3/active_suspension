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

%% ----------------- Gain Scaling Levels -----------------
scales = [0.1, 0.5, 1.0, 2.0, 5.0];

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

    % Calculate T_plot once, using simulation_time
    T_plot = 0:resolution:simulation_time;

    % Process CRG file
    [road_length, ~, road_profile] = process_crg_file(crg_file, T_plot, simulation_speed);

    %% ----------------- State-Space Model -----------------
    [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);

    fprintf('\n===== CRG Profile: %s =====\n', crg_name);

    % Loop through each controller
    for ctrl_idx = 1:numel(controllers)
        controller = controllers{ctrl_idx};
        controller_func = str2func(['run_', lower(controller), '_control']);

        % Check if gains exist for the current controller and CRG profile
        if ~isfield(gain_map, controller) || ~isfield(gain_map.(controller), crg_name)
            warning('Gains for %s in CRG-%s not found. Skipping.', controller, crg_name);
            continue;
        end

        % Get the base gains for the current controller and CRG profile
        gains = gain_map.(controller).(crg_name);

        fprintf('\n----- Controller: %s -----\n', controller); % New header for controller

        % Run controller-specific simulation and print base gains
        switch controller
            case 'Hinf'
                fprintf('Base Gains for Hinf Controller (%s):\n', crg_name);
                disp('Q Base Matrix:');
                disp(gains{1});
                fprintf('Gamma Base: %.4f\n', gains{2});
            case 'LQR'
                fprintf('Base Gains for LQR Controller (%s):\n', crg_name);
                disp('Q Base Matrix:');
                disp(gains{1});
                fprintf('R Base: %.4f\n', gains{2});
            case 'SMC'
                fprintf('Base Gains for SMC Controller (%s): [%.4f, %.4f]\n', crg_name, gains{1}, gains{2});
            case 'PID'
                fprintf('Base Gains for PID Controller (%s): [%.4f, %.4f, %.4f]\n', crg_name, gains{1}, gains{2}, gains{3});
            otherwise
                fprintf('Unknown controller.\n');
                continue;
        end

        % Simulate controller for different gain scales
        all_results = simulate_controller(simulation_speed, scales, gains, controller_func, A, B, T_plot, road_profile, x0, controller, crg_name);

        %% ----------------- Display Performance Metrics -----------------
        road_title = crg_name;
        title_str = sprintf('Gain Sweep - %s - %s', controller, road_title);
        display_metrics_table(title_str, all_results);

        %% ----------------- Plot Results -----------------
        if exist('T_plot', 'var') && exist('road_profile', 'var')
            plot_results(T_plot * simulation_speed, road_profile, title_str, road_title, all_results, plot_flag, crg_name);
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

function all_results = simulate_controller(simulation_speed, scales, baseGains, controller_func, A, B, T, u, x0, controller, crg_name)
    all_results = struct();
    for scale = scales
        switch controller
            case 'LQR'
                % For LQR: scale only R
                scaledGains = {baseGains{1}, baseGains{2} * scale};

            case 'Hinf'
                % For H-Infinity: scale only gamma
                scaledGains = {baseGains{1}, baseGains{2} * scale};

            otherwise
                % For PID, SMC : scale all gain matrices
                scaledGains = cellfun(@(g) g * scale, baseGains, 'UniformOutput', false);
        end

        label = sprintf('%s - %s - x%.1f', controller, crg_name, scale);
        key = sprintf('%s_%s_x%.1f', controller, crg_name, scale);
        key = matlab.lang.makeValidName(key);

        % Run the controller simulation
        try
            [t, y] = controller_func(A, B, T, u, x0, scaledGains);
            all_results.(key).t = t;
            all_results.(key).y = y;
            all_results.(key).label = label;
            all_results.(key).speed = simulation_speed;
        catch err
            warning(err.identifier, 'Simulation failed: %s', err.message);
            all_results.(key).t = T;
            all_results.(key).y = NaN(numel(T), size(B, 2));
            all_results.(key).label = [label, ' (error)'];
            all_results.(key).speed = simulation_speed;
        end

        % Print scaled gains
        printScaledGains(scale, scaledGains, crg_name, controller);
    end
end

function printScaledGains(scale, scaledGains, crg_name, controller_type)
    fprintf('Scaled Gains for %s Controller (%s) (%.1fx):', controller_type, crg_name, scale);
    switch controller_type
        case 'Hinf'
            fprintf('\nQ Scaled Matrix:\n');
            disp(scaledGains{1});
            fprintf('Gamma Scaled: %.4f\n', scaledGains{2});
        case 'LQR'
            fprintf('\nQ Scaled Matrix:\n');
            disp(scaledGains{1});
            fprintf('R Scaled: %.4f\n', scaledGains{2});
        case 'SMC'
            fprintf(' [%.4f, %.4f]\n', scaledGains{1}, scaledGains{2});
        case 'PID'
            fprintf(' [%.4f, %.4f, %.4f]\n', scaledGains{1}, scaledGains{2}, scaledGains{3});
        otherwise
            fprintf(' [Unknown format]\n');
    end
end
