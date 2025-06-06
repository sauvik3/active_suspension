clc;
clear;
close all;

%% ----------------- Setup Path -----------------
current_script_dir = fileparts(mfilename('fullpath'));
path_parts = strsplit(current_script_dir, filesep);
sim_idx = find(strcmp(path_parts, 'simulation'), 1, 'last');

if ~isempty(sim_idx)
    base_path = strjoin(path_parts(1:sim_idx), filesep);
else
    % Fallback if 'simulation' not found in path, assumes current_script_dir is base_path
    warning('train_road_classifier:BasePathNotFound', ...
            'Could not find "simulation" folder. Assuming current script directory as base path.');
    base_path = current_script_dir;
end

disp(['Base Path for Training: ', base_path]);

% Add necessary subdirectories to MATLAB path
sub_dirs = {'utils/ml', 'ml_models'};
for i = 1:numel(sub_dirs)
    full_path = fullfile(base_path, sub_dirs{i});
    if exist(full_path, 'dir')
        addpath(full_path);
    else
        warning('Missing path: %s', full_path);
    end
end

fprintf('\n--- Loading Dataset Files from Multiple PVS Folders ---\n');

%% ----------------- Step 1: Data Acquisition and Loading -----------------
% Define the list of PVS folders to process
pvs_folders = {'PVS 1', 'PVS 2', 'PVS 3', 'PVS 4', 'PVS 5', 'PVS 6', 'PVS 7', 'PVS 8', 'PVS 9'};

% Initialize containers for all data across all PVS folders
all_features = []; % To store all extracted features
all_labels = {};   % To store corresponding road type labels

% Define column names (consistent across all PVS files)
timestamp_col_mpu = 'timestamp';
accel_z_col = 'acc_z_above_suspension';

% Define the list of boolean label columns to be used for classification
% Make sure these are exact matches to your dataset_labels.csv header
boolean_label_cols = {'paved_road', 'unpaved_road', 'dirt_road', ...
                      'cobblestone_road', 'asphalt_road', 'no_speed_bump', ...
                      'speed_bump_asphalt', 'speed_bump_cobblestone', ...
                      'good_road_left', 'regular_road_left', 'bad_road_left', ...
                      'good_road_right', 'regular_road_right', 'bad_road_right'};

% Duration = 1.0 second / segment for alignment with likely 1Hz labels (e.g., GPS rate).
segment_duration_seconds = 1.0;

for pvs_idx = 1:numel(pvs_folders)
    current_pvs_folder_name = pvs_folders{pvs_idx};
    dataset_folder = fullfile(base_path, 'ml_dataset', current_pvs_folder_name);

    fprintf('\nProcessing folder: %s\n', current_pvs_folder_name);

    if ~exist(dataset_folder, 'dir')
        warning('RoadClassifier:FolderNotFound', 'Dataset folder not found at %s. Skipping this folder.', dataset_folder);
        continue; % Skip to the next folder
    end

    try
        % Load MPU data (assuming 'dataset_gps_mpu_left.csv' is preferred)
        mpu_left_data = readtable(fullfile(dataset_folder, 'dataset_gps_mpu_left.csv'));
        % Load labels (this will now be the boolean flags file)
        labels_data = readtable(fullfile(dataset_folder, 'dataset_labels.csv'));

        % Check if essential columns exist in MPU data
        if ~ismember(timestamp_col_mpu, mpu_left_data.Properties.VariableNames) || ...
           ~ismember(accel_z_col, mpu_left_data.Properties.VariableNames)
            warning('train_road_classifier:MPUColMissing', ...
                    'Required columns (%s, %s) not found in MPU data for %s. Skipping this folder.', ...
                    timestamp_col_mpu, accel_z_col, current_pvs_folder_name);
            continue;
        end

        % Check if essential boolean label columns exist
        missing_label_cols = setdiff(boolean_label_cols, labels_data.Properties.VariableNames);
        if ~isempty(missing_label_cols)
            warning('train_road_classifier:LabelColMissing', ...
                    'Boolean label column(s) "%s" not found in dataset_labels.csv for %s. Skipping this folder.', ...
                    strjoin(missing_label_cols, ', '), current_pvs_folder_name);
            continue;
        end

        %% ----------------- Step 2: Data Preprocessing and Synchronization -----------------
        % Determine average sampling resolution from MPU data
        time_resolution_mpu = mean(diff(mpu_left_data.(timestamp_col_mpu)));
        if isnan(time_resolution_mpu) || time_resolution_mpu == 0
            warning('train_road_classifier:TimeRes', ...
                    'Could not determine MPU time resolution for %s. Assuming 0.01s (100 Hz).', current_pvs_folder_name);
            time_resolution_mpu = 0.01; % Fallback resolution
        end
        fprintf('  Detected MPU Time Resolution: %.4f s (approx. %.1f Hz)\n', time_resolution_mpu, 1/time_resolution_mpu);

        % Calculate number of MPU data points per segment
        rows_per_segment = round(segment_duration_seconds / time_resolution_mpu);
        if rows_per_segment < 2 % Need at least 2 points for diff and features
            warning('train_road_classifier:SmallSegment', ...
                    'Segment duration is too short for MPU resolution in %s. Skipping this folder.', current_pvs_folder_name);
            continue;
        end
        fprintf('  Each segment will be approx. %d MPU data points (%.2f seconds).\n', rows_per_segment, segment_duration_seconds);

        % Loop through MPU data in fixed-size windows
        num_mpu_rows = size(mpu_left_data, 1);
        num_label_rows = size(labels_data, 1);
        % The number of segments we can analyze is limited by both available MPU data and available labels.
        % The MPU data is segmented into `segment_duration_seconds` windows.
        % The labels data is assumed to have one row per `segment_duration_seconds` interval,
        % aligning directly by row index.
        num_segments = min(floor(num_mpu_rows / rows_per_segment), num_label_rows);

        if num_segments == 0
            warning('train_road_classifier:NoSegments', ...
                    'No segments can be created for %s. Check MPU data length, labels data length, and segment_duration_seconds.', ...
                    current_pvs_folder_name);
            continue;
        end
        fprintf('  Attempting to create %d segments for feature extraction.\n', num_segments);

        for i = 1:num_segments
            % Determine start and end row for the current MPU segment
            start_row_mpu = (i - 1) * rows_per_segment + 1;
            end_row_mpu = start_row_mpu + rows_per_segment - 1;

            if end_row_mpu > num_mpu_rows
                % This break is technically covered by num_segments calculation, but good for safety.
                break;
            end

            segment_accel_z = mpu_left_data.(accel_z_col)(start_row_mpu:end_row_mpu);

            % Get the corresponding boolean label row for the current segment
            % This assumes a 1:1 alignment between MPU segments and label rows.
            current_boolean_label_row = labels_data(i, boolean_label_cols); % Use table slicing to get relevant columns

            if ~isempty(segment_accel_z) && length(segment_accel_z) >= 2 % At least 2 points needed for diff and features
                %% ----------------- Step 3: Feature Extraction -----------------
                current_features_struct = extract_road_features(segment_accel_z, time_resolution_mpu);

                % Convert the feature struct to a row vector.
                % The order of fields MUST be consistent with how the ML model expects them during prediction!
                current_features_vector = [
                    current_features_struct.Mean, ...
                    current_features_struct.Std, ...
                    current_features_struct.Variance, ...
                    current_features_struct.Max, ...
                    current_features_struct.Min, ...
                    current_features_struct.RMS, ...
                    current_features_struct.Skewness, ...
                    current_features_struct.Kurtosis, ...
                    current_features_struct.Energy, ...
                    current_features_struct.PeakToPeak, ...
                    current_features_struct.CrestFactor
                ];

                % This function applies the priority logic to derive a single class label.
                derived_label = map_boolean_labels_to_single_class(current_boolean_label_row);

                % Optionally skip 'Unknown' labeled segments or if no label is active
                if ~strcmp(derived_label, 'Unknown')
                    all_features = [all_features; current_features_vector]; %#ok<AGROW>
                    all_labels = [all_labels; {derived_label}]; %#ok<AGROW>
                end
            else
                % fprintf('Skipping segment %d: not enough MPU data points or invalid.\n', i); % Uncomment for debugging
            end
        end

    catch ME
        warning('train_road_classifier:ProcessError', ...
                'Error processing folder %s: %s. Skipping this folder.', current_pvs_folder_name, ME.message);
        continue;
    end
end

if isempty(all_features)
    error('train_road_classifier:NoFeatures', ...
          'No features could be extracted from any dataset. Check folder paths, data availability, and segment_duration_seconds.');
end

all_labels_categorical = categorical(all_labels);
fprintf('\n--- Aggregated Data Summary ---\n');
fprintf('Total effective segments processed from all folders: %d\n', size(all_features, 1));
fprintf('Extracted features size: %d x %d\n', size(all_features));
fprintf('Classes used for training: %s\n', strjoin(cellstr(unique(all_labels_categorical)), ', '));
fprintf('Class distribution:\n');
disp(countcats(all_labels_categorical));

% --- Print feature value statistics (for first few features) ---
fprintf('\n--- Feature Distribution Overview (First 5 Features) ---\n');
num_feat_to_show = min(5, size(all_features, 2));
for fi = 1:num_feat_to_show
    f = all_features(:, fi);
    fprintf('  Feature %2d: Mean = %.4f | Std = %.4f | Min = %.4f | Max = %.4f\n', ...
            fi, mean(f), std(f), min(f), max(f));
end

%% ----------------- Step 4: Model Training -----------------
fprintf('\n--- Training Machine Learning Model ---\n');

% Split data into training and testing sets (70% train, 30% test)
% Ensure there's enough data for all classes in both train/test sets, or handle with care.
if numel(unique(all_labels_categorical)) < 2 || size(all_features, 1) < 10
    warning('train_road_classifier:NotEnoughData', ...
            'Not enough unique classes or samples for robust train/test split. Training with all data for demonstration.');
    XTrain = all_features;
    YTrain = all_labels_categorical;
    XTest = []; % No separate test set
    YTest = [];
else
    cv = cvpartition(all_labels_categorical, 'Holdout', 0.3);
    XTrain = all_features(training(cv), :);
    YTrain = all_labels_categorical(training(cv));
    XTest = all_features(test(cv), :);
    YTest = all_labels_categorical(test(cv));
end

fprintf('Training data size: %d samples\n', size(XTrain, 1));
if ~isempty(XTest)
    fprintf('Testing data size: %d samples\n', size(XTest, 1));

    % --- Display label distribution in train/test ---
    fprintf('\nTrain/Test Class Distribution:\n');
    u_classes = categories(all_labels_categorical);
    for i = 1:numel(u_classes)
        c = u_classes{i};
        n_train = sum(YTrain == c);
        n_test = sum(YTest == c);
        fprintf('  %-25s : Train = %3d | Test = %3d\n', c, n_train, n_test);
    end
else
    fprintf('No separate testing data (due to insufficient samples).\n');
end

% Train the classifier (e.g., Support Vector Machine - SVM)
try
    roadClassifierModel = fitcsvm(XTrain, YTrain, ...
                                   'KernelFunction', 'linear', ...
                                   'Standardize', true, ...
                                   'ClassNames', unique(all_labels_categorical));
    fprintf('SVM Model Trained Successfully.\n');
catch ME
    error('train_road_classifier:ModelTrainingError', ...
          'Error during model training: %s', ME.message);
end

%% ----------------- Step 5: Model Evaluation -----------------
fprintf('\n--- Evaluating Model Performance ---\n');

if ~isempty(XTest)
    YPred = predict(roadClassifierModel, XTest);
    accuracy = sum(YPred == YTest) / numel(YTest);
    fprintf('Model Accuracy on Test Set: %.2f%%\n', accuracy * 100);

    % Display confusion matrix (requires Statistics and Machine Learning Toolbox)
    try
        figure('Name', 'Confusion Matrix', 'NumberTitle', 'off');
        cm = confusionchart(YTest, YPred);
        cm.Title = 'Road Classifier Confusion Matrix';
        cm.RowSummary = 'row-normalized'; % Show precision (or recall based on axis)
        cm.ColumnSummary = 'column-normalized'; % Show recall (or precision)
        fprintf('Confusion Matrix displayed.\n');
    catch ME
        if strcmp(ME.identifier, 'MATLAB:UndefinedFunction')
            warning('train_road_classifier:ConfusionChartMissing', ...
                    'Confusion chart requires Statistics and Machine Learning Toolbox. Skipping plot.');
        else
            warning('train_road_classifier:ConfusionChartError', ...
                    'Error displaying confusion chart: %s', ME.message);
        end
    end
else
    fprintf('No separate test set for evaluation.\n');
    % If no test set, you can evaluate on training set for a quick check (but not true performance)
    % YPredTrain = predict(roadClassifierModel, XTrain);
    % accuracyTrain = sum(YPredTrain == YTrain) / numel(YTrain);
    % fprintf('Accuracy on Training Set: %.2f%%\n', accuracyTrain * 100);
end

%% ----------------- Step 6: Model Saving -----------------
fprintf('\n--- Saving Trained Model ---\n');

ml_models_folder = fullfile(base_path, 'ml_models');
if ~exist(ml_models_folder, 'dir')
    mkdir(ml_models_folder); % Create the folder if it doesn't exist
    fprintf('Created folder: %s\n', ml_models_folder);
end

model_save_path = fullfile(ml_models_folder, 'road_classifier_model.mat');
try
    save(model_save_path, 'roadClassifierModel');
    fprintf('Trained ML model saved successfully to: %s\n', model_save_path);
catch ME
    error('train_road_classifier:ModelSaveError', ...
          'Error saving model: %s', ME.message);
end

disp('Road Classifier Model Training Process Complete!');