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

% Simulation Parameters
displacement_test_values = [-0.0225, -0.01, 0.0, 0.0055, 0.0075, 0.0225];    % [m]
speed_test_values =        [20, 60, 80, 85, 90];                             % [m/s]
controllers = {'PID', 'SMC', 'LQR', 'Hinf'};

% Run the tests
for j = 1:numel(controllers)
    ctrl = controllers{j};
    test_gain_interpolation(ctrl, displacement_test_values, speed_test_values, plot_flag);
end

%% ----------------- Test Driver Function -----------------
function test_gain_interpolation(controller, displacement_test_values, speed_test_values, plot_flag)
    % Perform Lookup and Store Results
    gain_results = cell(length(displacement_test_values), length(speed_test_values));
    for i = 1:length(displacement_test_values)
        for j = 1:length(speed_test_values)
            gain_results{i, j} = gain_interpolate(controller, ...
                displacement_test_values(i), speed_test_values(j));
        end
    end

    % Print Results
    printGainLookupResults(controller, speed_test_values, displacement_test_values, gain_results);

    % Plot Results
    plotGainSurface(controller, speed_test_values, displacement_test_values, gain_results, plot_flag, controller);
end

%% ----------------- Print Results Function -----------------
function printGainLookupResults(controller, speed_vals, disp_vals, gain_results)
    fprintf('Gain Lookup Results (%s):\n', controller);
    fprintf('-----------------------------\n');
    for i = 1:length(disp_vals)
        fprintf('  Displacement: %.3f m\n', disp_vals(i));
        for j = 1:length(speed_vals)
            gain = gain_results{i, j};
            fprintf('    Speed: %.2f m/s\n', speed_vals(j));
            if strcmp(controller, 'LQR') || strcmp(controller, 'Hinf')
                fprintf('    Q Base Matrix:\n');
                disp(gain.Q);
                fprintf('    R Base: %.4f\n', gain.R);
            elseif strcmp(controller, 'PID')
                fprintf('    Gain (Kp, Ki, Kd): [%.2f, %.2f, %.2f]\n', gain(1), gain(2), gain(3));
            elseif strcmp(controller, 'SMC')
                fprintf('    Gain (Ks, epsilon): [%.2f, %.2f]\n', gain(1), gain(2));
            else
                fprintf('    Gain: %s\n', mat2str(gain));
            end
        end
    end
    fprintf('-----------------------------\n\n');
end

%% ----------------- Plot Function -----------------
function plotGainSurface(controller, speed_vals, disp_vals, gain_results, plot_flag, varargin)
    % Determine figure visibility based on plot_flag
    figure_visibility = 'off'; % Default to invisible
    if plot_flag == 1 || plot_flag == 3 % If plotting is desired (Plot or Plot+Export)
        figure_visibility = 'on'; % Make figure visible
    end

    if strcmp(controller, 'LQR') || strcmp(controller, 'Hinf')
        labels = {'Q_11', 'Q_22', 'Q_33', 'Q_44', 'R'};
        n_plots = 5;
    else
        n_plots = size(gain_results{1, 1}, 2);
        if strcmp(controller, 'PID')
            labels = {'Kp', 'Ki', 'Kd'};
        elseif strcmp(controller, 'SMC')
            labels = {'Ks', 'epsilon'};
        else
            labels = arrayfun(@(i) sprintf('Gain %d', i), 1:n_plots, 'UniformOutput', false);
        end
    end

    [S_mesh, D_mesh] = meshgrid(speed_vals, disp_vals);

    for k = 1:n_plots
        Z = zeros(size(S_mesh));
        for i = 1:size(D_mesh, 1)
            for j = 1:size(D_mesh, 2)
                gain = gain_results{i, j};
                if strcmp(controller, 'LQR') || strcmp(controller, 'Hinf')
                    if k <= 4
                        q_diag = diag(gain.Q);
                        Z(i, j) = q_diag(k);
                    else
                        Z(i, j) = gain.R;
                    end
                else
                    Z(i, j) = gain(k);
                end
            end
        end
        % Create the figure
        fig = figure('Name', labels{k}, 'NumberTitle', 'off', 'Visible', figure_visibility);

        surf(S_mesh, D_mesh, Z);
        title_str = sprintf('%s vs Speed & Displacement (%s)', labels{k}, controller);
        title(title_str, 'Interpreter', 'none');
        xlabel('Speed (m/s)');
        ylabel('Displacement (m)');
        zlabel(labels{k});
        grid on;
        colorbar;
        view(135, 30);

        % --- Saving the figure based on plot_flag ---
        if plot_flag == 2 || plot_flag == 3 % If export is desired (Export or Plot+Export)
            if ~isempty(varargin)
                save_figure(fig, title_str, varargin{:});
            else
                save_figure(fig, title_str);
            end
        end

        % --- Closing the figure based on plot_flag ---
        % Close figure if only exporting or no plotting is desired
        if plot_flag == 0 || plot_flag == 2
            close(fig);
        end
        % If plot_flag is 1 (Plot) or 3 (Plot+Export), the figure remains open for user viewing.
    end
end
