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

% Load parameter file
param_file = fullfile(base_path, 'init', 'parameters.m');
disp(['Looking for parameters.m at: ', param_file]);
if exist(param_file, 'file')
    run(param_file);
else
    error('Missing file: %s', param_file);
end

% Define simulation time vector (if not already defined in parameters.m)
if ~exist('T', 'var')
    T = 0:0.001:10;
end

%% ----------------- Process CRG File -----------------
crg_folder = fullfile(base_path, 'crg_dataset/ASAM_OpenCRG_BS_V1.2.0/crg-bin');
crg_file = fullfile(crg_folder, 'country_road.crg');

[~, crg_fname, crg_ext] = fileparts(crg_file);
fprintf('\nProcessing CRG File: %s%s\n', crg_fname, crg_ext);

[road_length, resolution, road_profile] = process_crg_file(crg_file, T, simulation_speed);

%% ----------------- Lateral Profile Simulation -----------------
crg_lateral_variation(crg_file);

%% ----------------- Get Base Path -----------------
function base_path = get_base_path()
    if endsWith(pwd, 'simulation')
        base_path = pwd;
    else
        base_path = fullfile(pwd, 'simulation');
    end
end

%% ----------------- Lateral Variation Plotting -----------------
function crg_lateral_variation(crg_file)
    data = crg_read(crg_file);
    [~, crg_fname, crg_ext] = fileparts(crg_file);
    crg_title = [crg_fname, crg_ext];

    if isfield(data.head, 'vmin') && isfield(data.head, 'vmax') && data.head.vmax > data.head.vmin
        v_values = linspace(data.head.vmin, data.head.vmax, 100);
    else
        warning('Invalid or missing lateral bounds in CRG. Using default range [-0.75, 0.75] m.');
        v_values = linspace(-0.75, 0.75, 100);
    end

    if isfield(data.head, 'ubeg') && isfield(data.head, 'uend') && data.head.uend > data.head.ubeg
        s_values = linspace(data.head.ubeg, data.head.uend, 2000);
    else
        error('Invalid longitudinal bounds in CRG header.');
    end

    Z_map = zeros(length(v_values), length(s_values));

    for i = 1:length(v_values)
        uv = [s_values; v_values(i)*ones(size(s_values))]';
        z = crg_eval_uv2z(data, uv);
        Z_map(i, :) = z;
    end

    plot_lateral_profiles(s_values, v_values, Z_map, crg_title);
    plot_3d_surface(s_values, v_values, Z_map, crg_title);
end

function plot_lateral_profiles(s_values, v_values, Z_map, crg_title)
    selected_indices = round(linspace(1, length(v_values), 5));
    f1 = figure('Name', ['Lateral Tracks: ', crg_title], 'NumberTitle', 'off');
    hold on;
    colors = lines(length(selected_indices));
    for i = 1:length(selected_indices)
        vi = selected_indices(i);
        plot(s_values, Z_map(vi, :), 'Color', colors(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('v = %.2f m', v_values(vi)));
    end
    xlabel('Longitudinal Distance (m)');
    ylabel('Elevation (m)');
    title(['CRG Elevation Tracks - ', crg_title], 'Interpreter', 'none');
    legend('show');
    grid on;
end

function plot_3d_surface(s_values, v_values, Z_map, crg_title)
    f2 = figure('Name', ['3D Surface: ', crg_title], 'NumberTitle', 'off');
    [S, V] = meshgrid(s_values, v_values);
    surf(S, V, Z_map, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    xlabel('Longitudinal Distance (m)');
    ylabel('Lateral Position (m)');
    zlabel('Elevation (m)');
    title(['CRG 3D Road Surface - ', crg_title], 'Interpreter', 'none');
    colormap turbo;
    colorbar;
    view(45, 30);
    axis tight;
    grid on;
end
