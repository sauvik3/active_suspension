clc;
clear;
close all;

%% ----------------- Setup Simulation -----------------
% Ensure correct base path
base_path = get_base_path();
disp(['Base Path: ', base_path]);

% Initialize OpenCRG dataset
crg_init_script = fullfile(base_path, 'crg_dataset', 'ASAM_OpenCRG_BS_V1.2.0', 'matlab', 'crg_init.m');
if exist(crg_init_script, 'file')
    run(crg_init_script);
else
    warning('OpenCRG initialization script not found: %s', crg_init_script);
end

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
disp(['Looking for parameters.m at: ', param_file]);

if exist(param_file, 'file')
    run(param_file);
else
    error('Missing file: %s', param_file);
end

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

%% ----------------- Active Control Simulations -----------------
controllers = {'Passive', 'PID', 'SMC', 'LQR', 'Hinf'};
results = struct();
road_type = 'CRG_country_road';  % Label for CRG-based gains

for j = 1:numel(controllers)
    ctrl = controllers{j};
    if strcmp(ctrl, 'Passive')
        [results.(ctrl).t, results.(ctrl).y] = run_passive_control(A, B, T_plot, road_profile, x0);
        results.(ctrl).speed = simulation_speed;
    elseif isfield(gains_lookup().CRG, ctrl) && isfield(gains_lookup().CRG.(ctrl), road_type)
        gains = gains_lookup().CRG.(ctrl).(road_type);
        func_handle = str2func(['run_', lower(ctrl), '_control']);

        % Reset initial state for each controller
        x0_ctrl = x0;
        [results.(ctrl).t, results.(ctrl).y] = func_handle(A, B, T_plot, road_profile, x0_ctrl, gains);
        results.(ctrl).speed = simulation_speed;
    else
        warning('Gains not found for %s under CRG profile', ctrl);
        results.(ctrl).t = [];
        results.(ctrl).y = [];
        results.(ctrl).speed = simulation_speed;
    end
end

%% ----------------- Display Performance Metrics -----------------
display_metrics_table(sprintf('CRG: %s', road_type), results);

%% ----------------- Plot Results -----------------
road_title = road_type;
title_str = sprintf('Simulation - %s', road_title);

if exist('T_plot', 'var') && exist('road_profile', 'var')
    plot_results(T_plot * simulation_speed, road_profile, title_str, road_title, results, plot_flag);
else
    warning('T_plot or road_profile not defined for plotting.');
end
fprintf('------------------------------\n');

%% ----------------- Get Base Path -----------------
function base_path = get_base_path()
    if endsWith(pwd, 'simulation')
        base_path = pwd;
    else
        base_path = fullfile(pwd, 'simulation');
    end
end
