function gain = gain_interpolate(controller, displacement, speed)
    % Define the lookup table.
    [gains_table, disp_vals, speed_vals] = defineLookupTable();
    gain_cell_array = gains_table.(controller);
    n_disp = length(disp_vals);
    n_speed = length(speed_vals);

    % Clamp input values to within the defined grid bounds
    displacement = min(max(displacement, min(disp_vals)), max(disp_vals));
    speed       = min(max(speed,       min(speed_vals)), max(speed_vals));

    if strcmp(controller, 'LQR') || strcmp(controller, 'Hinf')
        Q_diags = zeros(n_disp, n_speed, 4);  % Preallocate for Q diagonals
        R_vals  = zeros(n_disp, n_speed);     % Preallocate for R values

        % Extract Q diagonals and R
        for i = 1:n_disp
            for j = 1:n_speed
                entry    = gain_cell_array{i}{j};
                Q_diag   = diag(entry{1});
                Q_diags(i, j, :) = Q_diag;
                R_vals(i, j)      = entry{2};
            end
        end

        % Interpolate
        Q_interp_diag = zeros(4, 1);
        for k = 1:4
            Q_interp_diag(k) = interp2(speed_vals, disp_vals, Q_diags(:, :, k), speed, displacement, 'linear', NaN);
        end
        gain.Q = diag(Q_interp_diag);
        gain.R = interp2(speed_vals, disp_vals, R_vals, speed, displacement, 'linear', NaN);
    else % PID or SMC
        num_gain_params = size(gain_cell_array{1}{1}, 2);
        gain_matrices      = NaN(n_disp, n_speed, num_gain_params);

        for i = 1:n_disp
            for j = 1:n_speed
                gain_vec = cell2mat(gain_cell_array{i}{j});
                gain_matrices(i, j, :) = gain_vec;
            end
        end
        gain = zeros(1, num_gain_params);
        for k = 1:num_gain_params
            gain(k) = interp2(speed_vals, disp_vals, gain_matrices(:, :, k), speed, displacement, 'linear', NaN);
        end
    end
end

function [gains_table, displacement_values, speed_values] = defineLookupTable()
    displacement_values = [-0.03, -0.015, 0, 0.015, 0.03];  % [m]
    speed_values = [10, 30, 50, 70, 90];                    % [m/s]

    gains_table = struct( ...
        'PID', {{ ...
            {{1280, 16, 270}, {1420, 18, 295}, {1560, 21, 330}, {1680, 24, 355}, {1780, 26, 370}}, ...   % -0.03 m
            {{1480, 19, 290}, {1620, 23, 320}, {1760, 28, 355}, {1880, 33, 385}, {1980, 36, 405}}, ...   % -0.015 m
            {{1580, 23, 315}, {1720, 27, 345}, {1850, 32, 375}, {1960, 36, 400}, {2050, 38, 415}}, ...   % 0 m
            {{1670, 26, 335}, {1810, 30, 365}, {1940, 34, 395}, {2050, 37, 420}, {2140, 39, 435}}, ...   % 0.015 m
            {{1750, 28, 350}, {1890, 33, 380}, {2020, 37, 410}, {2130, 40, 435}, {2220, 42, 450}}  ...   % 0.03 m
        }}, ...
        'SMC', {{ ...
            {{780, 0.22}, {830, 0.27}, {880, 0.31}, {920, 0.33}, {960, 0.35}},       ...   % -0.03 m
            {{880, 0.26}, {930, 0.31}, {980, 0.36}, {1030, 0.39}, {1080, 0.42}},     ...   % -0.015 m
            {{990, 0.34}, {1040, 0.39}, {1090, 0.44}, {1140, 0.47}, {1190, 0.48}},   ...   % 0 m
            {{1080, 0.33}, {1130, 0.38}, {1180, 0.43}, {1230, 0.46}, {1280, 0.49}},  ...   % 0.015 m
            {{1160, 0.31}, {1210, 0.36}, {1260, 0.41}, {1310, 0.45}, {1360, 0.47}}   ...   % 0.03 m
        }}, ...
        'LQR', {{ ...
            {{diag([980, 48, 2900, 45]), 4.7}, {diag([1020, 50, 3000, 47]), 5.1}, {diag([1070, 52, 3120, 50]), 5.5}, {diag([1110, 55, 3250, 53]), 6}, {diag([1160, 58, 3350, 55]), 6.4}},           ...   % -0.03 m
            {{diag([1180, 57, 3450, 56]), 5.8}, {diag([1220, 60, 3580, 60]), 6.1}, {diag([1270, 63, 3700, 63]), 6.6}, {diag([1310, 66, 3820, 66]), 7.1}, {diag([1360, 68, 3940, 68]), 7.4}},        ...   % -0.015 m
            {{diag([1400, 70, 4050, 70]), 6.9}, {diag([1440, 73, 4170, 72]), 7.2}, {diag([1490, 75, 4280, 74]), 7.6}, {diag([1530, 78, 4400, 77]), 8.1}, {diag([1580, 80, 4520, 80]), 8.3}},        ...   % 0 m
            {{diag([1620, 82, 4620, 82]), 7.9}, {diag([1660, 85, 4740, 84]), 8.2}, {diag([1710, 87, 4860, 86]), 8.7}, {diag([1750, 90, 4980, 89]), 9.1}, {diag([1800, 92, 5100, 92]), 9.4}},        ...   % 0.015 m
            {{diag([1820, 91, 5200, 94]), 9.0}, {diag([1860, 94, 5320, 96]), 9.3}, {diag([1910, 96, 5440, 98]), 9.7}, {diag([1950, 99, 5560, 100]), 10.2}, {diag([2000, 100, 5680, 102]), 10.5}}    ...   % 0.03 m
        }}, ...
        'Hinf', {{ ...
            {{diag([7500, 190, 7000, 90]), 1.8}, {diag([8000, 210, 7400, 110]), 2.1}, {diag([8600, 235, 7800, 130]), 2.5}, {diag([9200, 260, 8200, 160]), 2.8}, {diag([9800, 285, 8600, 190]), 3.1}},           ... % -0.03 m
            {{diag([8200, 220, 7800, 110]), 2.1}, {diag([8800, 245, 8200, 135]), 2.4}, {diag([9400, 270, 8600, 165]), 2.7}, {diag([10000, 295, 9000, 195]), 3.0}, {diag([10600, 320, 9400, 220]), 3.3}},        ... % -0.015 m
            {{diag([9000, 250, 8500, 135]), 2.4}, {diag([9600, 275, 8900, 160]), 2.7}, {diag([10200, 300, 9300, 190]), 3.0}, {diag([10800, 325, 9700, 215]), 3.3}, {diag([11400, 350, 10100, 240]), 3.6}},      ... % 0 m
            {{diag([9800, 275, 9100, 165]), 2.7}, {diag([10400, 300, 9500, 190]), 3.0}, {diag([11000, 325, 9900, 215]), 3.3}, {diag([11600, 350, 10300, 240]), 3.6}, {diag([12200, 375, 10700, 265]), 3.9}},    ... % 0.015 m
            {{diag([10600, 300, 9700, 195]), 3.0}, {diag([11200, 325, 10100, 220]), 3.3}, {diag([11800, 350, 10500, 245]), 3.6}, {diag([12400, 375, 10900, 270]), 3.9}, {diag([13000, 400, 11300, 295]), 4.2}}  ... % 0.03 m
        }} ...
    );
end
