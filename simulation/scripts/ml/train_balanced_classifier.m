% Train a multi-class SVM classifier using a balanced dataset

clc;
clear;
close all;

%% --- Configuration --------------------------------------------------
data_file = fullfile(pwd, 'ml_dataset', 'combined_dataset.csv');
model_save_path = fullfile(pwd, 'ml_models', 'road_classifier_model.mat');

if ~isfile(data_file)
    error('Dataset file not found at: %s', data_file);
end

%% --- Load and Sanitize Dataset --------------------------------------
fprintf('--- Loading Dataset ---\n');
data = readtable(data_file);

assert(any(strcmp('ClassLabel', data.Properties.VariableNames)), ...
    'ClassLabel column missing in dataset.');

X = data{:, 1:end-1};
Y = categorical(data.ClassLabel);  % force categorical

% Remove samples with NaN values
nan_rows = any(isnan(X), 2);
if any(nan_rows)
    warning('Removed %d rows containing NaNs.', sum(nan_rows));
    X(nan_rows, :) = [];
    Y(nan_rows) = [];
end

fprintf('Loaded %d samples with %d features.\n', size(X,1), size(X,2));
disp_class_counts(Y);

%% --- Plot Class Distribution ----------------------------------------
plot_class_distribution(Y, 'Overall Class Distribution');

%% --- Plot Sample Feature Distributions ------------------------------
num_features_to_plot = min(3, size(X,2));
for i = 1:num_features_to_plot
    plot_feature_distribution(X(:,i), Y, i);
end

%% --- Train/Test Split -----------------------------------------------
cv = cvpartition(Y, 'HoldOut', 0.2);
XTrain = X(training(cv), :);
YTrain = Y(training(cv), :);
XTest  = X(test(cv), :);
YTest  = Y(test(cv), :);

%% --- Plot Train/Test Distribution -----------------------------------
plot_train_test_split(YTrain, YTest);

%% --- Train Classifier -----------------------------------------------
fprintf('\n--- Training Classifier ---\n');
t = templateSVM('Standardize', true);
roadClassifierModel = fitcecoc(XTrain, YTrain, 'Learners', t, 'Coding', 'onevsall');

%% --- Save Model -----------------------------------------------------
if ~exist(fileparts(model_save_path), 'dir')
    mkdir(fileparts(model_save_path));
end
save(model_save_path, 'roadClassifierModel');
fprintf('Trained model saved to: %s\n', model_save_path);

%% --- Evaluate Model -------------------------------------------------
fprintf('\n--- Evaluating on Test Set ---\n');
YPred = predict(roadClassifierModel, XTest);

conf_mat = confusionmat(YTest, YPred);
acc = mean(YPred == YTest);

fprintf('Test Accuracy: %.2f%%\n', acc * 100);
disp('Confusion Matrix:');
disp(conf_mat);

plot_confusion_heatmap(conf_mat, categories(Y));

%% --- Utility Functions ----------------------------------------------
function disp_class_counts(Y)
    fprintf('Class Distribution:\n');
    summary = countcats(Y);
    labels = categories(Y);
    for i = 1:numel(summary)
        fprintf('  %-10s: %d\n', string(labels{i}), summary(i));
    end
end

function plot_class_distribution(Y, plot_title)
    figure('Name', plot_title, 'NumberTitle', 'off');
    bar(countcats(Y));
    xticklabels(categories(Y));
    xlabel('Class');
    ylabel('Sample Count');
    title(plot_title);
    grid on;
end

function plot_feature_distribution(feature, labels, index)
    figure('Name', sprintf('Feature %d Distribution', index), 'NumberTitle', 'off');
    boxplot(feature, labels);
    xlabel('Class');
    ylabel(sprintf('Feature %d Value', index));
    title(sprintf('Feature %d Distribution by Class', index));
    grid on;
end

function plot_train_test_split(YTrain, YTest)
    train_counts = countcats(YTrain);
    test_counts  = countcats(YTest);
    figure('Name', 'Train/Test Distribution', 'NumberTitle', 'off');
    bar([train_counts, test_counts]);
    legend('Train', 'Test');
    xticklabels(categories(YTrain));
    xlabel('Class');
    ylabel('Sample Count');
    title('Train vs Test Class Distribution');
    grid on;
end

function plot_confusion_heatmap(conf_mat, labels)
    figure('Name', 'Confusion Matrix', 'NumberTitle', 'off');
    imagesc(conf_mat);
    colormap('hot');
    colorbar;
    axis square;
    xticks(1:numel(labels));
    yticks(1:numel(labels));
    xticklabels(labels);
    yticklabels(labels);
    xlabel('Predicted Class');
    ylabel('True Class');
    title('Confusion Matrix Heatmap');
end
