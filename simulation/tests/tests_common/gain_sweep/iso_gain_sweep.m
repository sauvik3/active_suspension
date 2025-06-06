clc;
clear;
close all;

%% ----------------- Setup Paths -----------------
if endsWith(pwd, 'simulation')
    base_path = pwd;
else
    base_path = fullfile(pwd, 'simulation');
end

sub_dirs = {'algo', 'controller', 'init', 'utils'};
for i = 1:numel(sub_dirs)
    addpath(fullfile(base_path, sub_dirs{i}));
end

param_file = fullfile(base_path, 'init', 'parameters.m');
if exist(param_file, 'file')
    run(param_file);
else
    error('Missing file: %s', param_file);
end

%% ----------------- Controller Setup -----------------
controllers = {'PID', 'SMC', 'LQR', 'Hinf'};
gain_map = gains_lookup().ISO;

%% ----------------- Gain Scaling Levels -----------------
scales = [0.1, 0.5, 1.0, 2.0, 5.0];

% Get ISO classes
iso_classes = fieldnames(gain_map.(controllers{1}));

% Loop through each ISO class
for i = 1:numel(iso_classes)
    iso_class = iso_classes{i};

    % Calculate T_plot once per ISO class, using simulation_time
    T_plot = 0:resolution:simulation_time;

    % Retrieve road profile and state space matrices
    class_psd = class_psd_values(i);
    u = generate_road_profile(road_len, resolution, T_plot, class_psd);
    [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);

    fprintf('\n===== ISO Class: %s =====\n', iso_class);

    % Loop through each controller
    for ctrl_idx = 1:numel(controllers)
        controller = controllers{ctrl_idx};
        controller_func = str2func(['run_', lower(controller), '_control']);

        % Ensure gains exist for the current controller and ISO class
        if ~isfield(gain_map, controller) || ~isfield(gain_map.(controller), iso_class)
            warning('Gains for %s in ISO-%s not found. Skipping.', controller, iso_class);
            continue;
        end

        % Get the base gains for the current controller and ISO class
        gains = gain_map.(controller).(iso_class);

        % Run controller-specific simulation and print base gains
        fprintf('\n===== Controller: %s =====\n', controller);
        switch controller
            case 'Hinf'
                fprintf('Base Gains for Hinf Controller (ISO-%s):\n', iso_class);
                disp('Q Base Matrix:');
                disp(gains{1});
                fprintf('Gamma Base: %.4f\n', gains{2});
            case 'LQR'
                fprintf('Base Gains for LQR Controller (ISO-%s):\n', iso_class);
                disp('Q Base Matrix:');
                disp(gains{1});
                fprintf('R Base: %.4f\n', gains{2});
            case 'SMC'
                fprintf('Base Gains for SMC Controller (ISO-%s): [%.4f, %.4f]\n', iso_class, gains{1}, gains{2});
            case 'PID'
                fprintf('Base Gains for PID Controller (ISO-%s): [%.4f, %.4f, %.4f]\n', iso_class, gains{1}, gains{2}, gains{3});
            otherwise
                fprintf('Unknown controller.\n');
                continue;
        end

        % Simulate controller for different gain scales
        all_results = simulate_controller(simulation_speed, scales, gains, controller_func, A, B, T_plot, u, x0, controller, iso_class);

        %% ----------------- Display Performance Metrics -----------------
        road_title = sprintf('ISO-%s', iso_class);
        title_str = sprintf('Gain Sweep - %s - %s', controller, road_title);
        display_metrics_table(title_str, all_results);

        %% ----------------- Plot Results -----------------
        plot_results(T_plot * simulation_speed, u, title_str, road_title, all_results, plot_flag, controller);
        fprintf('------------------------------\n');
    end
end

%% ----------------- Local Functions -----------------
function all_results = simulate_controller(simulation_speed, scales, baseGains, controller_func, A, B, T_ref, u_ref, x0, controller, iso_class)
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

        label = sprintf('%s - ISO Profile %s - x%.1f', controller, iso_class, scale);
        key = sprintf('%s_ISO_%s_x%.1f', controller, iso_class, scale);
        key = matlab.lang.makeValidName(key);

        % Run the controller simulation
        try
            [all_results.(key).t, all_results.(key).y] = controller_func(A, B, T_ref, u_ref, x0, scaledGains);
            all_results.(key).label = label;
            all_results.(key).speed = simulation_speed;
        catch err
            warning(err.identifier, 'Simulation failed: %s', err.message);
            all_results.(key).t = T_ref;
            all_results.(key).y = NaN(numel(T_ref), size(B, 2));
            all_results.(key).label = [label, ' (error)'];
            all_results.(key).speed = simulation_speed;
        end

        % Print scaled gains
        printScaledGains(scale, scaledGains, iso_class, controller);
    end
end

function printScaledGains(scale, scaledGains, iso_class, controller_type)
    fprintf('Scaled Gains for %s Controller (ISO-%s) (%.1fx):', controller_type, iso_class, scale);
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
