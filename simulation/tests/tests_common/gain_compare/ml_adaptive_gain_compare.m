clc;
clear;
close all;

%% ----------------- Setup Path -----------------
base_path = get_base_path();
disp(['Base Path: ', base_path]);

sub_dirs = {'algo', 'controller', 'init', 'utils', 'utils/ml', 'ml_models'};
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

% Verify and load parameter file
run(fullfile(base_path, 'init', 'parameters.m'));

% --- Simulation Parameters ---
fprintf('\n--- Initial System Parameters ---\n');
fprintf('Sprung Mass (m_s): %.4f kg\n', m_s);
fprintf('Unsprung Mass (m_u): %.4f kg\n', m_u);
fprintf('Sprung Stiffness (k_s): %.4f N/m\n', k_s);
fprintf('Tire Stiffness (k_t): %.4f N/m\n', k_t);
fprintf('Sprung Damping (b_s): %.4f Ns/m\n', b_s);
fprintf('Simulation Resolution: %.4f s\n', resolution);
fprintf('Road Length (configured): %.4f m\n', road_len);
fprintf('Initial State (x0): [%s]\n', num2str(x0));
fprintf('--------------------------------------\n');

%% ----------------- ML Model Setup -----------------
% Load the gain map (this now primarily contains CRG gains, and ML section is for reference/display)
% Assuming gains_lookup is also available on path
gain_map = gains_lookup();

% Load the pre-trained ML classification model.
% This model is used to predict road type for informational display,
% but adaptive gains are now determined by 2D interpolation.
ml_model_file = fullfile(base_path, 'ml_models', 'road_classifier_model.mat');
roadClassifierModel = []; % Initialize
if exist(ml_model_file, 'file')
    load(ml_model_file, 'roadClassifierModel');
    fprintf('ML Road Classifier Model Loaded from: %s\n', ml_model_file);
else
    relative_model_path = fullfile('ml_models', 'road_classifier_model.mat');
    warning('ml_adaptive_gain_compare:ModelNotFound', 'ML Road Classifier Model not found at %s. Road type classification for display will use a simple heuristic.', relative_model_path);
end

% Define the controllers to be tested in the simulation sweep
controllers = {'PID', 'SMC', 'LQR', 'Hinf'};

%% ----------------- Speed Sweep Parameters -----------------
speeds_kmh = [10, 30, 50, 70, 90]; % Speeds in km/h for the sweep
speeds_mps = speeds_kmh / 3.6;     % Convert speeds to m/s

%% ----------------- Simulation Loop (Speed Sweep) -----------------
crg_name = 'CRG_country_road'; % Name of the CRG road profile used for testing

% Define path to the CRG file
crg_folder = fullfile(base_path, 'crg_dataset', 'ASAM_OpenCRG_BS_V1.2.0', 'crg-bin');
crg_file = fullfile(crg_folder, 'country_road.crg');

[~, crg_fname, crg_ext] = fileparts(crg_file);
fprintf('\nProcessing CRG File: %s%s\n', crg_fname, crg_ext);

% Calculate max simulation time for the slowest speed to ensure full road profile generation
max_sim_time_for_plot = road_len / min(speeds_mps);
[actual_crg_length, ~, road_profile_base] = process_crg_file(crg_file, 0:resolution:max_sim_time_for_plot, min(speeds_mps));

fprintf('Actual CRG Data Length: %.4f m\n', actual_crg_length);
if abs(actual_crg_length - road_len) > 1e-3
    warning('ml_adaptive_gain_compare:RoadLengthMismatch', 'Configured road_len (%.2f m) does not match actual CRG data length (%.2f m). Simulation will extrapolate/truncate.', road_len, actual_crg_length);
end

fprintf('\n===== ML Adaptive Gain Test - CRG Profile: %s - Speed Sweep =====\n', crg_name);

% Loop through each controller type
for ctrl_idx = 1:numel(controllers)
    ctrl = controllers{ctrl_idx};
    controller_func = str2func(['run_', lower(ctrl), '_control']);

    % Get state-space matrices for the current system parameters
    [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);

    fprintf('\n----- Controller: %s -----\n', ctrl);

    % Loop through each speed for the current controller
    for s_idx = 1:numel(speeds_mps)
        current_speed_mps = speeds_mps(s_idx);
        current_speed_kmh = speeds_kmh(s_idx);
        fprintf('\n--- Speed: %.1f km/h (%.2f m/s) ---\n', current_speed_kmh, current_speed_mps);

        % Calculate simulation time and time vector for current speed
        simulation_time = road_len / current_speed_mps;
        T_current_speed = 0:resolution:simulation_time;

        % Interpolate road profile for the current speed
        road_profile_current_speed = interp1((0:length(road_profile_base)-1) * (min(speeds_mps) * resolution), ...
                                              road_profile_base, ...
                                              T_current_speed * current_speed_mps, 'linear', 'extrap');

        comparison_results_for_this_speed = struct();

        %% --- Baseline Simulation: Fixed Gains ---
        % Use gains from the 'CRG' section of gain_map for fixed-gain simulation
        fixed_gains = gain_map.CRG.(ctrl).(crg_name);
        try
            [t_fixed, y_fixed] = controller_func(A, B, T_current_speed, road_profile_current_speed, x0, fixed_gains);
            comparison_results_for_this_speed.FixedGain.t = t_fixed;
            comparison_results_for_this_speed.FixedGain.y = y_fixed;
            comparison_results_for_this_speed.FixedGain.label = sprintf('%s - Fixed Gain', ctrl);
            comparison_results_for_this_speed.FixedGain.speed = current_speed_mps;
            fprintf('    RMS Sprung Mass Disp (Fixed Gain): %.10f m\n', rms(y_fixed(:,1)));
        catch fixed_err
            warning(fixed_err.identifier, 'Fixed gain simulation failed for %s at %.1f km/h: %s', ...
                                ctrl, current_speed_kmh, fixed_err.message);
            comparison_results_for_this_speed.FixedGain.t = T_current_speed;
            num_states = size(A, 1);
            comparison_results_for_this_speed.FixedGain.y = NaN(numel(T_current_speed), num_states);
            comparison_results_for_this_speed.FixedGain.label = sprintf('%s - Fixed Gain (Error)', ctrl);
            comparison_results_for_this_speed.FixedGain.speed = current_speed_mps;
        end

        %% --- ML Adaptive Gain Simulation ---
        predicted_road_type = 'Unknown';
        try
            % Calculate sprung mass acceleration from fixed-gain simulation results
            raw_sprung_accel = diff(y_fixed(:,2)) / resolution;

            % Ensure acceleration vector has same length as displacement vector
            target_length = length(y_fixed(:,2));
            sim_sprung_accel = zeros(target_length, 1);
            sim_sprung_accel(1:target_length-1) = raw_sprung_accel;
            sim_sprung_accel(target_length) = raw_sprung_accel(end); % Copy last value to match length

            if ~isempty(sim_sprung_accel) && length(sim_sprung_accel) > 1
                % Extract features from simulated sprung mass acceleration
                features_vector = extract_road_features(sim_sprung_accel, resolution);

                % Predict road type using the loaded ML model or heuristic
                if ~isempty(roadClassifierModel)
                    expected_model_features = size(roadClassifierModel.X, 2);
                    if size(features_vector, 2) ~= expected_model_features
                        warning('ml_adaptive_gain_compare:FeatureMismatch', ...
                                'Feature vector size (%d) does not match ML model expected features (%d). Attempting to pad/truncate.', ...
                                size(features_vector, 2), expected_model_features);
                        % Pad with zeros or truncate to match expected features
                        temp_features = zeros(1, expected_model_features);
                        num_copy = min(size(features_vector, 2), expected_model_features);
                        temp_features(1, 1:num_copy) = features_vector(1, 1:num_copy);
                        features_vector = temp_features;
                    end

                    predicted_road_type = char(predict(roadClassifierModel, features_vector));
                else
                    % Heuristic for display if ML model is not available
                    current_rms_disp_fixed_gain = rms(y_fixed(:,1));
                    if current_rms_disp_fixed_gain < 0.00002251
                        predicted_road_type = 'Asphalt Road';
                    elseif current_rms_disp_fixed_gain < 0.00005
                        predicted_road_type = 'Cobblestone Road';
                    else
                        predicted_road_type = 'Dirt Road';
                    end
                    fprintf('    (Using heuristic for road classification due to missing ML model.)\n');
                end
                fprintf('    Simulated Road Type Classified as: %s\n', predicted_road_type);
            else
                warning('ml_adaptive_gain_compare:NoSimulatedSensorData', 'Insufficient simulated sensor data for feature extraction. Cannot classify road type.');
                predicted_road_type = 'Unknown';
            end

            % Use gain_interpolate based on RMS displacement and current speed
            % The rms(y_fixed(:,1)) is the RMS of the sprung mass displacement from the fixed-gain simulation.
            % This RMS value, along with speed, drives the gain interpolation.
            displacement_input = rms(y_fixed(:,1));

            % Call the gain_interpolate function
            % It returns a struct (for LQR/Hinf) or a vector (for PID/SMC)
            interpolated_gain_output = gain_interpolate(ctrl, displacement_input, current_speed_mps);

            % Convert the output of gain_interpolate into a cell array format
            if isstruct(interpolated_gain_output) % For LQR/Hinf: returns a struct with Q and R
                if isfield(interpolated_gain_output, 'Q') && isfield(interpolated_gain_output, 'R')
                    adaptive_gains = {interpolated_gain_output.Q, interpolated_gain_output.R};
                elseif isfield(interpolated_gain_output, 'gamma')
                    adaptive_gains = {interpolated_gain_output.gamma};
                else
                    error('ml_adaptive_gain_compare:InvalidStructGains', 'Struct gains format not recognized.');
                end
            else % For PID/SMC: returns a numeric vector, which needs to be unpacked
                 % into individual cell elements for fixed-signature controllers.
                if strcmp(ctrl, 'PID')
                    % PID gains are {Kp, Ki, Kd}
                    if numel(interpolated_gain_output) == 3
                        adaptive_gains = {interpolated_gain_output(1), interpolated_gain_output(2), interpolated_gain_output(3)};
                    else
                        error('ml_adaptive_gain_compare:InvalidPIDGains', 'PID gain vector from interpolation has unexpected number of elements.');
                    end
                elseif strcmp(ctrl, 'SMC')
                    % SMC gains are {k, epsilon}
                    if numel(interpolated_gain_output) == 2
                        adaptive_gains = {interpolated_gain_output(1), interpolated_gain_output(2)};
                    else
                        error('ml_adaptive_gain_compare:InvalidSMCGains', 'SMC gain vector from interpolation has unexpected number of elements.');
                    end
                else
                    % Fallback for other potential non-LQR/Hinf controllers
                    % If they also return a vector, but expect it as a single cell:
                    adaptive_gains = {interpolated_gain_output};
                    warning('ml_adaptive_gain_compare:UnknownControllerFormat', 'Controller %s expects a single cell array for its gain vector. Verify this is correct.', ctrl);
                end
            end

            % Check for NaN/Inf in adaptive_gains
            if any(cellfun(@(x) any(isnan(x(:))), adaptive_gains)) || any(cellfun(@(x) any(isinf(x(:))), adaptive_gains))
                error('ml_adaptive_gain_compare:InvalidAdaptiveGains', 'Interpolated adaptive gains contain NaN or Inf values. This will likely cause simulation failure.');
            end

            fprintf('    ML Adaptive Gain (for %s) using 2D interpolated gains.\n', ctrl);

            % Run simulation with the newly interpolated adaptive gains
            try
                [t_ml_adaptive, y_ml_adaptive] = controller_func(A, B, T_current_speed, road_profile_current_speed, x0, adaptive_gains);
            catch sim_err
                warning('ml_adaptive_gain_compare:MLAdaptiveSimFailed', ...
                        'ML Adaptive simulation failed for %s at %.1f km/h: %s', ...
                        ctrl, current_speed_kmh, sim_err.message);
                % Rethrow the error to stop execution
                rethrow(sim_err);
            end

            comparison_results_for_this_speed.MLAdaptiveGain.t = t_ml_adaptive;
            comparison_results_for_this_speed.MLAdaptiveGain.y = y_ml_adaptive;
            comparison_results_for_this_speed.MLAdaptiveGain.label = sprintf('%s - ML Adaptive Gain', ctrl);
            comparison_results_for_this_speed.MLAdaptiveGain.speed = current_speed_mps;
            fprintf('    RMS Sprung Mass Disp (ML Adaptive Gain): %.10f m\n', rms(y_ml_adaptive(:,1)));

        catch ml_adaptive_top_err
            % This catch block will only be reached if the inner try-catch for controller_func
            % did not rethrow, or if an error occurred *before* the controller_func call.
            warning(ml_adaptive_top_err.identifier, 'ML Adaptive gain processing/simulation failed for %s at %.1f km/h (Outer Catch): %s', ...
                                 ctrl, current_speed_kmh, ml_adaptive_top_err.message);
            comparison_results_for_this_speed.MLAdaptiveGain.t = T_current_speed;
            num_states = size(A, 1);
            comparison_results_for_this_speed.MLAdaptiveGain.y = NaN(numel(T_current_speed), num_states);
            comparison_results_for_this_speed.MLAdaptiveGain.label = sprintf('%s - ML Adaptive Gain (Error)', ctrl);
            comparison_results_for_this_speed.MLAdaptiveGain.speed = current_speed_mps;
        end

        %% --- Plotting and Metrics for current speed ---
        title_str = sprintf('Gain Compare - %s - %s - %.1f kmph (ML Adaptive)', ctrl, crg_name, current_speed_kmh);
        display_metrics_table(title_str, comparison_results_for_this_speed);

        plot_results(T_current_speed * current_speed_mps, road_profile_current_speed, ...
                             title_str, crg_name, comparison_results_for_this_speed, plot_flag, ctrl);
    end
    fprintf('------------------------------\n');
end

%% ----------------- Get Base Path -----------------
function base_path = get_base_path()
    current_script_dir = fileparts(mfilename('fullpath'));
    path_parts = strsplit(current_script_dir, filesep);
    sim_idx = find(strcmp(path_parts, 'simulation'), 1, 'last');

    if ~isempty(sim_idx)
        base_path = strjoin(path_parts(1:sim_idx), filesep);
    else
        warning('get_base_path:NotFound', 'The "simulation" folder was not found in the current path. Using script''s parent directory as base path.');
        base_path = fileparts(current_script_dir);
    end
end