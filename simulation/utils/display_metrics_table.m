%% ----------------- Display Performance Metrics -----------------
function display_metrics_table(title_str, results)
    fprintf('\n=== Performance Metrics for %s ===\n', title_str);

    % Extract controller names
    controller_names = fieldnames(results);
    num_controllers = numel(controller_names);

    % Initialize table data
    settling_times = nan(num_controllers, 1);
    overshoots = nan(num_controllers, 1);
    rise_times = nan(num_controllers, 1);
    steady_state_errors = nan(num_controllers, 1);
    settled_status = strings(num_controllers, 1);

    % Compute and store metrics
    for i = 1:num_controllers
        controller = controller_names{i};
        if isfield(results, controller)
            t = results.(controller).t;
            y = results.(controller).y;
            % Pass 0 as the reference value, assuming equilibrium is at zero.
            [settling_time, overshoot, rise_time, sse] = compute_metrics(t, y, 0);

            settling_times(i) = settling_time;
            overshoots(i) = overshoot;
            rise_times(i) = rise_time;
            steady_state_errors(i) = sse;

            % Determine settled status (within a small tolerance of the end time)
            if ~isnan(settling_time) && abs(settling_time - t(end)) < 0.1
                settled_status(i) = "No";
            elseif isnan(settling_time)
                settled_status(i) = "N/A";
            else
                settled_status(i) = "Yes";
            end
        else
            settled_status(i) = "Error";
        end
    end

    % Define table headers
    headers = {'Controller', 'Settling Time (s)', 'Overshoot (%)', 'Rise Time (s)', 'Steady-State Error', 'Settled'};
    column_widths = [32, 20, 15, 15, 20, 10];
    separator_length = sum(column_widths) + numel(headers) - 1;

    % Print table header
    fprintf('%-*s', column_widths(1), headers{1});
    for i = 2:numel(headers)
        fprintf('%-*s', column_widths(i), headers{i});
    end
    fprintf('\n');
    fprintf('%s\n', repmat('-', 1, separator_length));

    % Print table body
    for i = 1:num_controllers
        fprintf('%-*s', column_widths(1), controller_names{i});
        fprintf('%-*.*f', column_widths(2), 2, settling_times(i));
        fprintf('%-*.*f', column_widths(3), 2, overshoots(i));
        fprintf('%-*.*f', column_widths(4), 2, rise_times(i));
        fprintf('%-*.*f', column_widths(5), 4, steady_state_errors(i));
        fprintf('%-*s', column_widths(6), settled_status(i));
        fprintf('\n');
    end
end
