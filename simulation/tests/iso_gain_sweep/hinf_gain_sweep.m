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
controller = 'Hinf';
controller_func = str2func(['run_', lower(controller), '_control']);
gain_map = gains_lookup().ISO;

if ~isfield(gain_map, controller)
    error('Gains for %s not found.', controller);
end

%% ----------------- Gain Scaling Levels -----------------
scales = [0.1, 0.5, 1.0, 2.0, 5.0];

%% ----------------- Loop Over ISO Road Profiles -----------------
road_classes = fieldnames(gain_map.(controller));

for i = 1:numel(road_classes)
    iso_class = road_classes{i};
    class_index = find(strcmp(iso_classes, iso_class));
    if isempty(class_index)
        warning('Skipping unknown ISO class: %s', iso_class);
        continue;
    end

    % Calculate T_plot once, using simulation_time
    T_plot = 0:resolution:simulation_time;

    class_psd = class_psd_values(class_index);
    u = generate_road_profile(road_len, resolution, T_plot, class_psd);
    [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);

    base = gain_map.(controller).(iso_class);
    Q_base = base{1};
    gamma_base = base{2};

    all_results = struct();

    fprintf('\n===== ISO Class %s =====\n', iso_class);

    for k = 1:numel(scales)
        scale = scales(k);
        gamma_scaled = gamma_base * scale;
        gains = {Q_base, gamma_scaled};  % Scale only gamma

        label = sprintf('%s - %s - gamma x%.1f', controller, iso_class, scale);
        key = matlab.lang.makeValidName(label);

        try
            fprintf('gamma scale: %.1f, R_hinf = %.4f\n', scale, gamma_scaled);

            [all_results.(key).t, all_results.(key).y] = controller_func(A, B, T_plot, u, x0, gains);
            all_results.(key).label = label;
            all_results.(key).speed = simulation_speed;
        catch err
            warning(err.identifier, 'Simulation failed: %s', err.message);
            all_results.(key).t = [];
            all_results.(key).y = [];
            all_results.(key).label = [label, ' (error)'];
            all_results.(key).speed = simulation_speed;
        end
    end

    %% ----------------- Plot Results -----------------
    road_title = sprintf('ISO-%s', iso_class);
    title_str = sprintf('Gain Sweep - %s - %s', controller, road_title);
    plot_results(T_plot * simulation_speed, u, title_str, road_title, all_results, plot_flag);
    fprintf('------------------------------\n');
end
