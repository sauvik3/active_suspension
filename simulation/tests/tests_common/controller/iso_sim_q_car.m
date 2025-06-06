clc;
clear;
close all;

%% ----------------- Setup Simulation -----------------
% Ensure correct base path
if endsWith(pwd, 'simulation')
    base_path = pwd;  % Already in 'simulation', no need to add it again
else
    base_path = fullfile(pwd, 'simulation');
end

disp(['Base Path: ', base_path]);

sub_dirs = {'algo', 'controller', 'init', 'utils'};

for i = 1:numel(sub_dirs)
    full_sub_path = fullfile(base_path, sub_dirs{i});
    disp(['Checking: ', full_sub_path]);
    if exist(full_sub_path, 'dir')
        addpath(full_sub_path);
    else
        warning('Path does not exist: %s', full_sub_path);
    end
end

% Verify and load parameter file
param_file = fullfile(base_path, 'init', 'parameters.m');
disp(['Looking for parameters.m at: ', param_file]);  % Debugging output

if exist(param_file, 'file')
    run(param_file);
else
    error('Missing file: %s', param_file);
end

%% ----------------- Loop over ISO road profiles -----------------
for i = 1:numel(iso_classes)
    iso_class = iso_classes{i};
    class_psd = class_psd_values(i);
    title_str = ['ISO Class ', iso_class];

    % Calculate T_plot once, using simulation_time
    T_plot = 0:resolution:simulation_time;

    % Generate road profile
    u = generate_road_profile(road_len, resolution, T_plot, class_psd);
    fprintf('\n===== %s =====\n', title_str);

    %% ----------------- State-Space Model -----------------
    [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);

    %% ----------------- Active Control Simulations -----------------
    controllers = {'Passive', 'PID', 'SMC', 'LQR', 'Hinf'};
    results = struct();

    for j = 1:numel(controllers)
        ctrl = controllers{j};
        if strcmp(ctrl, 'Passive')
            [results.(ctrl).t, results.(ctrl).y] = run_passive_control(A, B, T_plot, u, x0);
            results.(ctrl).speed = simulation_speed;
        elseif isfield(gains_lookup().ISO, ctrl) && isfield(gains_lookup().ISO.(ctrl), iso_class)
            gains = gains_lookup().ISO.(ctrl).(iso_class);
            func_handle = str2func(['run_', lower(ctrl), '_control']);

            % Reset initial state for each controller
            x0_ctrl = x0;
            [results.(ctrl).t, results.(ctrl).y] = func_handle(A, B, T_plot, u, x0_ctrl, gains);
            results.(ctrl).speed = simulation_speed;
        else
            warning('Gains not found for %s under ISO Class %s', ctrl, iso_class);
            results.(ctrl).t = [];
            results.(ctrl).y = [];
            results.(ctrl).speed = simulation_speed;
        end
    end

    %% ----------------- Display Performance Metrics -----------------
    display_metrics_table(title_str, results);

    %% ----------------- Plot Results -----------------
    road_title = title_str;

    if exist('T_plot', 'var') && exist('u', 'var')
        plot_results(T_plot * simulation_speed, u, title_str, road_title, results, plot_flag);
    else
        warning('T_plot or u not defined for plotting.');
    end
    fprintf('------------------------------\n');
end
