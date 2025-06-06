% generate_combined_dataset.m
% Extracts MPU sensor features, maps boolean labels to classes, and exports a combined dataset as CSV.

clc;
clear;
close all;

%% --- Configuration ---
base_path = fullfile(pwd, 'ml_dataset');  % Root folder for PVS datasets
output_path = fullfile(base_path, 'combined_dataset.csv');
balance_classes = true;                   % Enable/disable class balancing
segment_size = 50;                        % Segment length (in samples)

%% --- Initialization ---
pvs_folders = dir(fullfile(base_path, 'PVS *'));
combined_features = [];
combined_classes = [];

sensor_groups = {'dashboard', 'above_suspension', 'below_suspension'};
axes = {'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'};
stat_names = {'mean', 'std', 'peak', 'rms', 'skewness'};

fprintf('--- Loading Data from PVS Folders ---\n');

%% --- Process Each PVS Folder ---
for i = 1:length(pvs_folders)
    pvs_name = pvs_folders(i).name;
    folder_path = fullfile(base_path, pvs_name);

    labels_file = fullfile(folder_path, 'dataset_labels.csv');
    mpu_file = fullfile(folder_path, 'dataset_mpu_left.csv');

    if ~exist(labels_file, 'file') || ~exist(mpu_file, 'file')
        warning('Skipping %s: Missing required files.', pvs_name);
        continue;
    end

    labels = readtable(labels_file);
    mpu_data = readtable(mpu_file);

    if height(mpu_data) ~= height(labels)
        warning('Skipping %s: Row count mismatch (%d MPU, %d labels).', ...
                 pvs_name, height(mpu_data), height(labels));
        continue;
    end

    num_segments = floor(height(mpu_data) / segment_size);

    for s = 1:num_segments
        idx_start = (s - 1) * segment_size + 1;
        idx_end = s * segment_size;

        segment = mpu_data(idx_start:idx_end, :);
        label_row = labels(idx_end, :);  % Label at the end of the segment
        class_label = map_boolean_labels_to_single_class(label_row);

        % --- Extract Features for Each Sensor Axis ---
        feature_vector = [];
        for g = 1:length(sensor_groups)
            group = sensor_groups{g};
            for a = 1:length(axes)
                var_name = [axes{a} '_' group];
                if ismember(var_name, segment.Properties.VariableNames)
                    signal = segment.(var_name);
                    feature_vector = [feature_vector, ...
                        mean(signal), ...
                        std(signal), ...
                        max(abs(signal)), ...
                        rms(signal), ...
                        skewness(signal)];
                else
                    feature_vector = [feature_vector, NaN(1, numel(stat_names))];
                end
            end
        end

        combined_features = [combined_features; feature_vector];
        combined_classes = [combined_classes; string(class_label)];
    end

    fprintf('Processed %-6s | Segments: %4d\n', pvs_name, num_segments);
end

%% --- Dataset Summary ---
fprintf('\n--- Dataset Summary ---\n');
fprintf('Total segments: %d\n', size(combined_features, 1));

fprintf('\n--- Class Distribution ---\n');
disp(tabulate(categorical(combined_classes)));

%% --- Feature Naming ---
feature_names = {};
for g = 1:length(sensor_groups)
    for a = 1:length(axes)
        for s = 1:length(stat_names)
            feature_names{end+1} = sprintf('%s_%s_%s', axes{a}, sensor_groups{g}, stat_names{s});
        end
    end
end

%% --- Construct Final Table ---
dataset = array2table(combined_features, 'VariableNames', feature_names);
dataset.ClassLabel = categorical(combined_classes);

%% --- Optional Balancing ---
if balance_classes
    fprintf('\n--- Balancing Dataset ---\n');
    classes = categories(dataset.ClassLabel);
    min_count = min(groupcounts(dataset.ClassLabel));

    balanced_dataset = [];
    for k = 1:length(classes)
        class_k = classes{k};
        rows = dataset(dataset.ClassLabel == class_k, :);
        selected = rows(randperm(height(rows), min_count), :);
        balanced_dataset = [balanced_dataset; selected];
    end

    dataset = balanced_dataset;
    fprintf('Balanced to %d samples per class (Total: %d rows)\n', ...
             min_count, height(dataset));
end

%% --- Save to CSV ---
writetable(dataset, output_path);
fprintf('\nDataset saved to: %s\n', output_path);
