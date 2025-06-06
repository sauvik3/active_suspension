function features = extract_road_features(acceleration_signal, time_resolution)
    % EXTRACT_ROAD_FEATURES Calculates statistical features from an acceleration signal.
    %
    %   features = EXTRACT_ROAD_FEATURES(acceleration_signal, time_resolution)
    %
    %   Inputs:
    %     acceleration_signal - Numeric vector of acceleration values (time-series).
    %     time_resolution     - Scalar time step between samples (in seconds).
    %
    %   Outputs:
    %     features - Struct with statistical descriptors:
    %                .Mean, .Std, .Variance, .Max, .Min, .RMS,
    %                .Skewness, .Kurtosis, .Energy, .PeakToPeak, .CrestFactor

    % --- Input Validation ---
    if isempty(acceleration_signal) || ~isnumeric(acceleration_signal)
        error('acceleration_signal must be a non-empty numeric vector.');
    end
    if ~isscalar(time_resolution) || time_resolution <= 0
        error('time_resolution must be a positive scalar.');
    end

    % Ensure column vector
    if isrow(acceleration_signal)
        acceleration_signal = acceleration_signal';
    end

    % Basic statistics
    features.Mean      = mean(acceleration_signal);
    features.Std       = std(acceleration_signal);
    features.Variance  = var(acceleration_signal);
    features.Max       = max(acceleration_signal);
    features.Min       = min(acceleration_signal);

    % RMS
    features.RMS       = sqrt(mean(acceleration_signal.^2));

    % Shape-related statistics
    features.Skewness  = skewness(acceleration_signal);
    features.Kurtosis  = kurtosis(acceleration_signal);

    % Signal Energy (discrete approximation)
    features.Energy    = sum(acceleration_signal.^2) * time_resolution;

    % Peak-to-Peak amplitude
    features.PeakToPeak = features.Max - features.Min;

    % Crest Factor (peak magnitude / RMS)
    if features.RMS ~= 0
        features.CrestFactor = max(abs(acceleration_signal)) / features.RMS;
    else
        features.CrestFactor = NaN;
    end
end