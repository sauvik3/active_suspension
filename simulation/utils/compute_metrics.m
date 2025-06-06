%% ----------------- Performance Metrics -----------------
function [settling_time, overshoot, rise_time, steady_state_error] = compute_metrics(t, y, reference_value)
    % Compute performance metrics for a step response.
    % Inputs:
    %   t             - time vector
    %   y             - response signal (Nx1 or NxM)
    %   reference_value - target final value (default: 1)

    if nargin < 3
        reference_value = 1;  % Assume unit step input if not specified
    end

    y_out = y(:, 1); % Use first output dimension
    final_value = y_out(end);
    abs_final_value = abs(final_value);

    % Settling time: Last point within 15% of final value
    settling_threshold = 0.15 * abs_final_value;
    settling_indices = find(abs(y_out - final_value) <= settling_threshold, 1, 'last');
    settling_time = t(settling_indices);

    % Overshoot: Percentage above final value, capped at 200%
    peak_value = max(y_out);
    if final_value ~= 0
        overshoot = min(max(0, (peak_value - final_value) / abs_final_value * 100), 200);
    else
        overshoot = 0;
    end

    % Rise time: Time from 10% to 90% of final value
    if final_value ~= 0
        lower_bound = 0.1 * final_value;
        upper_bound = 0.9 * final_value;
        idx_10 = find(y_out >= lower_bound, 1, 'first');
        idx_90 = find(y_out >= upper_bound, 1, 'first');
        if ~isempty(idx_10) && ~isempty(idx_90)
            rise_time = t(idx_90) - t(idx_10);
        else
            rise_time = NaN;
        end
    else
        rise_time = NaN;
    end

    % Steady-state error: difference from reference, using average of last N points
    N = min(50, length(y_out));
    final_segment_avg = mean(y_out(end - N + 1:end));
    steady_state_error = reference_value - final_segment_avg;

    % Warn if output is still oscillating
    final_segment = y_out(end - N + 1:end); % Use the same segment for std
    if abs(reference_value) > eps && std(final_segment) > 0.01 * abs(reference_value)
        warning('Steady-state output may still be oscillating (std=%.4f)', std(final_segment));
    end
end
