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

% Verify and load parameter file
run(fullfile(base_path, 'init', 'parameters.m'));

% --- Initial Parameter Verification ---
fprintf('\n--- Initial System Parameters ---\n');
fprintf('Sprung Mass (m_s): %.4f kg\n', m_s);
fprintf('Unsprung Mass (m_u): %.4f kg\n', m_u);
fprintf('Sprung Stiffness (k_s): %.4f N/m\n', k_s);
fprintf('Tire Stiffness (k_t): %.4f N/m\n', k_t);
fprintf('Sprung Damping (b_s): %.4f Ns/m\n', b_s);
fprintf('Simulation Resolution: %.4f s\n', resolution);
fprintf('Road Length (configured): %.4f m\n', road_len);
fprintf('--------------------------------------\n');

%% ----------------- Controller Setup -----------------
controllers = {'PID'};
gain_map = gains_lookup().CRG;

%% ----------------- Speed Sweep Parameters -----------------
speeds_kmh = [10, 30, 50, 70, 90]; % Speeds in km/h
speeds_mps = speeds_kmh / 3.6;     % Convert to m/s

%% ----------------- Simulation Loop (Speed Sweep) -----------------
% Get CRG profiles
crg_profiles = fieldnames(gain_map.(controllers{1}));

% Loop through each CRG class
for i = 1:numel(crg_profiles)
    crg_name = crg_profiles{i};

    %% ----------------- Process CRG File -----------------
    crg_folder = fullfile(base_path, 'crg_dataset', 'ASAM_OpenCRG_BS_V1.2.0', 'crg-bin');
    crg_file = fullfile(crg_folder, 'country_road.crg');

    [~, crg_fname, crg_ext] = fileparts(crg_file);
    fprintf('\nProcessing CRG File: %s%s\n', crg_fname, crg_ext);

    % T_plot_base is calculated based on road_len from parameters.m
    T_plot_base = 0:resolution:(road_len / speeds_mps(1)); % Use smallest speed for max time base
    [actual_crg_length, ~, road_profile_base] = process_crg_file(crg_file, T_plot_base, speeds_mps(1));

    % Add actual CRG length to output for verification and warning if mismatch
    fprintf('Actual CRG Data Length: %.4f m\n', actual_crg_length);
    if abs(actual_crg_length - road_len) > 1e-3 % Check if actual CRG length significantly differs from configured road_len
        warning('crg_gain_compare:RoadLengthMismatch', 'Configured road_len (%.2f m) does not match actual CRG data length (%.2f m). Simulation will extrapolate/truncate.', road_len, actual_crg_length);
    end

    fprintf('\n===== CRG Profile: %s - Speed Sweep =====\n', crg_name);

    % Loop through each controller
    for ctrl_idx = 1:numel(controllers)
        ctrl = controllers{ctrl_idx};
        controller_func = str2func(['run_', lower(ctrl), '_control']);

        if ~isfield(gain_map, ctrl) || ~isfield(gain_map.(ctrl), crg_name)
            warning('Gains for %s in CRG-%s not found. Skipping controller.', ctrl, crg_name);
            continue;
        end

        [A, B] = get_state_space(m_s, m_u, k_s, k_t, b_s);

        fprintf('\n----- Controller: %s -----\n', ctrl);

        % Loop through each speed for the current CRG profile and controller
        for s_idx = 1:numel(speeds_mps)
            current_speed_mps = speeds_mps(s_idx);
            current_speed_kmh = speeds_kmh(s_idx);
            fprintf('\n--- Speed: %.1f km/h (%.2f m/s) ---\n', current_speed_kmh, current_speed_mps);

            simulation_time = road_len / current_speed_mps;
            T_current_speed = 0:resolution:simulation_time;

            % Ensure road_profile_current_speed matches T_current_speed length
            road_profile_current_speed = interp1(T_plot_base * speeds_mps(1), road_profile_base, T_current_speed * current_speed_mps);

            comparison_results_for_this_speed = struct();

            % --- Simulation with FIXED Gains ---
            fixed_gains = gain_map.(ctrl).(crg_name);

            try
                [t_fixed, y_fixed] = controller_func(A, B, T_current_speed, road_profile_current_speed, x0, fixed_gains);

                comparison_results_for_this_speed.FixedGain.t = t_fixed;
                comparison_results_for_this_speed.FixedGain.y = y_fixed;
                comparison_results_for_this_speed.FixedGain.label = sprintf('%s - Fixed Gain', ctrl);
                comparison_results_for_this_speed.FixedGain.speed = current_speed_mps;

                rms_sprung_disp = rms(y_fixed(:,1));
                fprintf('    RMS Sprung Mass Disp (Fixed Gain): %.10f m\n', rms_sprung_disp);

                % --- Simulation with INTERPOLATED Gains ---
                % Initialize as empty, type will depend on controller
                interpolated_gains_for_controller = [];

                try
                    % Call gain_interpolate. It returns a numeric array for PID/SMC, struct for LQR/Hinf.
                    raw_interpolated_gains = gain_interpolate(ctrl, rms_sprung_disp, current_speed_mps);

                    % Handle NaN values returned by gain_interpolate, and assign fallback as numeric array
                    if isnumeric(raw_interpolated_gains) && any(isnan(raw_interpolated_gains))
                        warning('crg_gain_compare:NaN_Gains', 'gain_interpolate returned NaN for %s at rms_sprung_disp = %.4f. Using default/fallback gains.', ctrl, rms_sprung_disp);
                        if strcmp(ctrl, 'PID')
                            raw_interpolated_gains = [1000, 10, 200]; % Fallback as numeric array
                        elseif strcmp(ctrl, 'SMC')
                            raw_interpolated_gains = [5000, 0.001]; % Fallback as numeric array
                        else
                            % For LQR/Hinf, if they return NaN (unlikely for a struct output)
                            % This case is less likely, but if it happens, fall back to fixed_gains which should be a struct
                            raw_interpolated_gains = fixed_gains; % Assume fixed_gains is compatible (struct)
                        end
                    end

                    % Now, convert raw_interpolated_gains into the final format expected by controller_func
                    if strcmp(ctrl, 'PID')
                        if numel(raw_interpolated_gains) == 3 && isnumeric(raw_interpolated_gains)
                            % Convert numeric array [Kp, Ki, Kd] to cell array {Kp, Ki, Kd}
                            interpolated_gains_for_controller = num2cell(raw_interpolated_gains);
                        else
                            error('crg_gain_compare:InvalidPIDGains', 'PID gains from gain_interpolate have unexpected format for cell array conversion.');
                        end
                    elseif strcmp(ctrl, 'SMC')
                        if numel(raw_interpolated_gains) == 2 && isnumeric(raw_interpolated_gains)
                            % Convert numeric array [Ks, epsilon] to cell array {Ks, epsilon}
                            interpolated_gains_for_controller = num2cell(raw_interpolated_gains);
                        else
                            error('crg_gain_compare:InvalidSMCGains', 'SMC gains from gain_interpolate have unexpected format for cell array conversion.');
                        end
                    elseif isstruct(raw_interpolated_gains)
                        % LQR/Hinf controllers expect a struct, but run_lqr_control might expect a cell array.
                        % Assuming run_lqr_control expects gains as a cell array {Q_matrix, R_scalar}
                        if isfield(raw_interpolated_gains, 'Q') && isfield(raw_interpolated_gains, 'R')
                            % Convert the struct fields into a cell array
                            interpolated_gains_for_controller = {raw_interpolated_gains.Q, raw_interpolated_gains.R};
                        else
                            error('crg_gain_compare:InvalidLQRHinfGains', 'LQR/Hinf gains struct from gain_interpolate is missing Q or R fields.');
                        end
                    else
                        error('crg_gain_compare:UnknownControllerConversion', 'Controller "%s" not handled for gain conversion. Expected struct or numeric array for conversion to cell.', ctrl);
                    end

                    % Now, call controller_func with the correctly formatted gains
                    [t_interp, y_interp] = controller_func(A, B, T_current_speed, road_profile_current_speed, x0, interpolated_gains_for_controller);

                    comparison_results_for_this_speed.InterpolatedGain.t = t_interp;
                    comparison_results_for_this_speed.InterpolatedGain.y = y_interp;
                    comparison_results_for_this_speed.InterpolatedGain.label = sprintf('%s - Interpolated Gain', ctrl);
                    comparison_results_for_this_speed.InterpolatedGain.speed = current_speed_mps;
                    fprintf('    RMS Sprung Mass Disp (Interp. Gain): %.10f m\n', rms(y_interp(:,1)));

                catch inter_err
                    warning(inter_err.identifier, 'Interpolated gain processing/simulation failed for %s at %.1f km/h: %s', ...
                                ctrl, current_speed_kmh, inter_err.message);
                    comparison_results_for_this_speed.InterpolatedGain.t = T_current_speed;
                    num_states = size(A, 1);
                    comparison_results_for_this_speed.InterpolatedGain.y = NaN(numel(T_current_speed), num_states);
                    comparison_results_for_this_speed.InterpolatedGain.label = sprintf('%s - Interp. Gain (Error)', ctrl);
                    comparison_results_for_this_speed.InterpolatedGain.speed = current_speed_mps;
                end

                % --- Plotting for current speed, comparing fixed vs. interpolated gains ---
                title_str = sprintf('Gain Compare - %s - %s - %.1f kmph', ctrl, crg_name, current_speed_kmh);
                display_metrics_table(title_str, comparison_results_for_this_speed);

                plot_results(T_plot_base * speeds_mps(1), road_profile_base, ...
                                 title_str, crg_name, comparison_results_for_this_speed, plot_flag, crg_name, ctrl);

            catch fixed_err
                warning(fixed_err.identifier, 'Fixed gain simulation failed for %s at %.1f km/h: %s', ...
                                ctrl, current_speed_kmh, fixed_err.message);
            end
        end
        fprintf('------------------------------\n');
    end
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