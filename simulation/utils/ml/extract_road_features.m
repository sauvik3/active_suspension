function features_vector = extract_road_features(acceleration_signal, time_resolution, num_segment_features)
% EXTRACT_ROAD_FEATURES Extracts a customizable feature vector from an acceleration signal.
%
%   features_vector = EXTRACT_ROAD_FEATURES(acceleration_signal, time_resolution)
%   features_vector = EXTRACT_ROAD_FEATURES(..., num_segment_features)
%
%   Inputs:
%     acceleration_signal     - Numeric vector of acceleration values (time-series).
%     time_resolution         - Scalar time step between samples (in seconds).
%     num_segment_features    - Optional. Either 14 or 24. Defaults to 24.
%
%   Outputs:
%     features_vector - 1xN numeric row vector of statistical features.

    % --- Input Validation ---
    if nargin < 3
        num_segment_features = 24;
    end

    if isempty(acceleration_signal) || ~isnumeric(acceleration_signal)
        error('acceleration_signal must be a non-empty numeric vector.');
    end
    if ~isscalar(time_resolution) || time_resolution <= 0
        error('time_resolution must be a positive scalar.');
    end
    if ~ismember(num_segment_features, [14, 24])
        error('num_segment_features must be 14 or 24.');
    end

    % Ensure column vector
    if isrow(acceleration_signal)
        acceleration_signal = acceleration_signal';
    end

    % --- Basic statistical features (11)
    basic_stats = [ ...
        mean(acceleration_signal), ...
        std(acceleration_signal), ...
        var(acceleration_signal), ...
        max(acceleration_signal), ...
        min(acceleration_signal), ...
        sqrt(mean(acceleration_signal.^2)), ... % RMS
        skewness(acceleration_signal), ...
        kurtosis(acceleration_signal), ...
        sum(acceleration_signal.^2) * time_resolution, ... % Energy
        max(acceleration_signal) - min(acceleration_signal) ... % Peak-to-Peak
    ];

    % Crest factor (peak/RMS)
    rms_val = basic_stats(6);
    crest_factor = max(abs(acceleration_signal)) / rms_val;
    if rms_val == 0
        crest_factor = NaN;
    end
    basic_stats(end+1) = crest_factor;

    % --- Segment-level features (14 or 24)
    seg_count = 8;
    L = length(acceleration_signal);
    pad_length = ceil(L / seg_count) * seg_count;
    padded_signal = [acceleration_signal; repmat(acceleration_signal(end), pad_length - L, 1)];
    reshaped_signal = reshape(padded_signal, [], seg_count);

    segment_features = zeros(1, seg_count * 3);  % Always compute full 24
    for i = 1:seg_count
        seg = reshaped_signal(:, i);
        idx = (i - 1) * 3 + 1;
        segment_features(idx:idx+2) = [mean(seg), std(seg), sqrt(mean(seg.^2))]; % mean, std, RMS
    end

    if num_segment_features == 14
        segment_features = segment_features(1:14);
    end

    % --- Frequency-domain features (8)
    Y = abs(fft(acceleration_signal));
    f_features = [ ...
        mean(Y), std(Y), var(Y), ...
        max(Y), min(Y), sum(Y), ...
        mean(diff(Y)), std(diff(Y)) ...
    ];

    % --- Combine all into final feature vector
    features_vector = [basic_stats, segment_features, f_features];

    % Optionally pad or trim (for consistency)
    expected_len = 11 + num_segment_features + 8;
    if length(features_vector) < expected_len
        features_vector(end+1:expected_len) = NaN;
    elseif length(features_vector) > expected_len
        features_vector = features_vector(1:expected_len);
    end
end