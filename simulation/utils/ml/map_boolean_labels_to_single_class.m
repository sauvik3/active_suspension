function single_label = map_boolean_labels_to_single_class(boolean_label_row)
    % MAP_BOOLEAN_LABELS_TO_SINGLE_CLASS Maps a row of boolean labels to a single categorical label.
    %
    %   single_label = MAP_BOOLEAN_LABELS_TO_SINGLE_CLASS(boolean_label_row)
    %
    %   Input:
    %       boolean_label_row - A table row (1xN table) with boolean (0 or 1) indicators.
    %
    %   Output:
    %       single_label - A char array representing the highest-priority label.
    %                      Returns 'Unknown' if no known label is active.

    % Defensive check: Only 1 row expected
    if height(boolean_label_row) ~= 1
        error('Input to map_boolean_labels_to_single_class must be a single row table.');
    end

    % Define priority mapping: {column_name, label_string}
    priority_list = {
        'speed_bump_cobblestone', 'Speed Bump Cobblestone';
        'speed_bump_asphalt',     'Speed Bump Asphalt';
        'cobblestone_road',       'Cobblestone Road';
        'dirt_road',              'Dirt Road';
        'asphalt_road',           'Asphalt Road';
        'unpaved_road',           'Unpaved Road';
        'paved_road',             'Paved Road';
        'good_road_left',         'Good Road Left';
        'good_road_right',        'Good Road Right';
        'regular_road_left',      'Regular Road Left';
        'regular_road_right',     'Regular Road Right';
        'bad_road_left',          'Bad Road Left';
        'bad_road_right',         'Bad Road Right';
        'no_speed_bump',          'No Speed Bump';
    };

    single_label = 'Unknown'; % Default

    for i = 1:size(priority_list, 1)
        col = priority_list{i, 1};
        label = priority_list{i, 2};

        if ismember(col, boolean_label_row.Properties.VariableNames)
            value = table2array(boolean_label_row(1, col));
            if value == 1
                single_label = label;
                return;
            end
        end
    end
end