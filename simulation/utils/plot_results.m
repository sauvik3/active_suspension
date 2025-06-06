%% ----------------- Plotting Function -----------------
function plot_results(T_ref, u_ref, title_str, road_title, all_results, plot_flag, varargin)
    % Determine figure visibility based on plot_flag
    figure_visibility = 'off'; % Default to invisible
    if plot_flag == 1 || plot_flag == 3 % If plotting is desired (Plot or Plot+Export)
        figure_visibility = 'on'; % Make figure visible
    end

    % Create the figure with specified visibility
    fig = figure('Name', title_str, 'NumberTitle', 'off', 'Visible', figure_visibility);
    tlo = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Plotting content (always done to populate the figure, even if invisible, for export purposes)
    nexttile(tlo);
    plot_road_profile(T_ref, u_ref, road_title);

    % Plot sprung mass displacement
    nexttile(tlo);
    plot_sprung_mass_displacement(all_results);

    % Plot unsprung mass displacement
    nexttile(tlo);
    plot_unsprung_mass_displacement(all_results);

    % --- Saving the figure based on plot_flag ---
    if plot_flag == 2 || plot_flag == 3 % If export is desired (Export or Plot+Export)
        full_subdir_path_components = {road_title}; 

        if numel(varargin) >= 2 
            full_subdir_path_components = [full_subdir_path_components, varargin(2:end)];
        elseif isscalar(varargin) && ~strcmp(varargin{1}, road_title)
            full_subdir_path_components = [full_subdir_path_components, varargin(1)];
        end

        sub_dir_string = fullfile(full_subdir_path_components{:});
        
        save_figure(fig, title_str, sub_dir_string);
    end

    % --- Closing the figure based on plot_flag ---
    % Close figure if only exporting or no plotting is desired
    if plot_flag == 0 || plot_flag == 2
        close(fig);
    end
    % If plot_flag is 1 (Plot) or 3 (Plot+Export), the figure remains open for user viewing.
end

%% Plot road profile
function plot_road_profile(T, u, title_str)
    plot(T, u, 'k', 'LineWidth', 1.2);
    title(['Road Profile - ', title_str], 'Interpreter', 'none');
    xlabel('Distance (m)');
    ylabel('Road Elevation (m)');
    grid on;
end

%% Plot sprung mass displacement
function plot_sprung_mass_displacement(all_results)
    hold on;
    plot_keys = fieldnames(all_results);
    colors = lines(numel(plot_keys));

    for i = 1:numel(plot_keys)
        key = plot_keys{i};
        res = all_results.(key);
        y = res.y;
        t_sim = res.t;

        if isempty(y) || isempty(t_sim)
            continue;
        end

        displayName = key;
        if isfield(res, 'label')
            displayName = res.label;
        end

        if size(t_sim, 1) ~= size(y, 1)
            warning('Time vector and state data have different lengths for %s', displayName);
            continue;
        end

        if isfield(res, 'speed')
            distance = t_sim * res.speed;
            plot(distance, y(:,1), 'DisplayName', displayName, 'Color', colors(i,:), 'LineWidth', 0.8);
        else
            warning('Speed information not found in results for %s. Plotting against time.', displayName);
            plot(t_sim, y(:,1), 'DisplayName', displayName, 'Color', colors(i,:), 'LineWidth', 0.8);
            xlabel('Time (s)');
        end
    end

    hold off;
    title('Sprung Mass Displacement');
    xlabel('Distance (m)');
    ylabel('Displacement (m)');
    legend('Location', 'best', 'Interpreter', 'none');
    grid on;
end

%% Plot unsprung mass displacement
function plot_unsprung_mass_displacement(all_results)
    hold on;
    plot_keys = fieldnames(all_results);
    colors = lines(numel(plot_keys));

    for i = 1:numel(plot_keys)
        key = plot_keys{i};
        res = all_results.(key);
        y = res.y;
        t_sim = res.t;

        if isempty(y) || isempty(t_sim)
            continue;
        end

        displayName = key;
        if isfield(res, 'label')
            displayName = res.label;
        end

        if size(t_sim, 1) ~= size(y, 1)
            warning('Time vector and state data have different lengths for %s', displayName);
            continue;
        end

        if isfield(res, 'speed')
            distance = t_sim * res.speed;
            plot(distance, y(:,2), 'DisplayName', displayName, 'Color', colors(i,:), 'LineWidth', 0.8);
        else
            warning('Speed information not found in results for %s. Plotting against time.', displayName);
            plot(t_sim, y(:,2), 'DisplayName', displayName, 'Color', colors(i,:), 'LineWidth', 0.8);
            xlabel('Time (s)');
        end
    end

    hold off;
    title('Unsprung Mass Displacement');
    xlabel('Distance (m)');
    ylabel('Displacement (m)');
    legend('Location', 'best', 'Interpreter', 'none');
    grid on;
end
