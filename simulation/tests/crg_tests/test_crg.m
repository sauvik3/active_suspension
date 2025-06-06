% Minimal OpenCRG Road Profile Processing
clc; clear; close all;

% Initialize OpenCRG dataset
base_path = get_base_path();
crg_init_script = fullfile(base_path, 'crg_dataset', 'ASAM_OpenCRG_BS_V1.2.0', 'matlab', 'crg_init.m');
if exist(crg_init_script, 'file')
    run(crg_init_script);
else
    warning('OpenCRG initialization script not found: %s', crg_init_script);
end

%% ----------------- Main Processing Function -----------------
function process_all_crg_files()
    base_path = get_base_path();
    crg_folder = fullfile(base_path, 'crg_dataset', 'ASAM_OpenCRG_BS_V1.2.0', 'crg-bin');
    crg_files = dir(fullfile(crg_folder, '*.crg'));
    
    if isempty(crg_files)
        disp('No CRG files found.');
        return;
    end
    
    for i = 1:length(crg_files)
        crg_file = fullfile(crg_folder, crg_files(i).name);
        try
            process_crg_file(crg_file);
        catch ME
            fprintf('Error processing %s: %s\n', crg_files(i).name, ME.message);
        end
    end
end

%% ----------------- Get Base Path -----------------
function base_path = get_base_path()
    if endsWith(pwd, 'simulation')
        base_path = pwd;
    else
        base_path = fullfile(pwd, 'simulation');
    end
end

%% ----------------- Process Single CRG File -----------------
function process_crg_file(crg_file)
    data = crg_read(crg_file);
    [s, z] = evaluate_crg(data);
    plot_crg_profile(s, z, crg_file);
    print_crg_info(data, crg_file);
end

%% ----------------- Evaluate CRG Profile -----------------
function [s, z] = evaluate_crg(data)
    s = linspace(data.head.ubeg, data.head.uend, 2000);  % Higher resolution
    v = zeros(size(s));
    z = crg_eval_uv2z(data, [s; v]');
end

%% ----------------- Plot CRG Profile -----------------
function plot_crg_profile(s, z, crg_file)
    [~, filename, ~] = fileparts(crg_file);
    fig = figure('Name', ['Figure: ', filename], 'NumberTitle', 'off');
    plot(s, z, 'b-', 'LineWidth', 1.5);
    xlabel('Distance (m)');
    ylabel('Elevation (m)');
    title(['CRG Road Profile: ', filename], 'Interpreter', 'none');
    grid on;
end

%% ----------------- Print CRG Information -----------------
function print_crg_info(data, crg_file)
    [~, filename, ~] = fileparts(crg_file);
    disp(['CRG File: ', filename]);
    disp(['CRG Resolution (uinc): ', num2str(data.head.uinc), ' m']);
    if isfield(data.head, 'vinc')
        disp(['CRG Resolution (vinc): ', num2str(data.head.vinc), ' m']);
    else
        disp('CRG Resolution (vinc): Not defined');
    end
    disp(['Total road length: ', num2str(data.head.uend - data.head.ubeg), ' m']);
end

crg_folder = fullfile(base_path, 'crg_dataset', 'ASAM_OpenCRG_BS_V1.2.0', 'crg-bin');
crg_file = fullfile(crg_folder, 'country_road.crg');
process_crg_file(crg_file);
