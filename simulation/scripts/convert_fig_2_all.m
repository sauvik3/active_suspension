% Get the directory of the current script
script_dir = fileparts(mfilename('fullpath'));

% Get the parent directory of the script directory (assuming 'figures' is a sibling to 'scripts')
parent_of_scripts_dir = fileparts(script_dir);

% Define source and output base folders relative to the script's parent directory
source_folder = fullfile(parent_of_scripts_dir, 'figures', 'fig');
output_base_folder = fullfile(parent_of_scripts_dir, 'figures');

% Define target formats
formats = {'png', 'svg'}; % Add more formats as needed, e.g., 'pdf', 'eps'

fprintf('Starting batch conversion of figures...\n');

% Recursively get all .fig files under the source directory
fig_files = dir(fullfile(source_folder, '**', '*.fig'));

if isempty(fig_files)
    fprintf('No .fig files found in %s to convert.\n', source_folder);
    disp('All conversions complete!');
    return;
end

% Loop through each .fig file found
for k = 1:numel(fig_files)
    fig_path = fullfile(fig_files(k).folder, fig_files(k).name);
    
    fprintf('  Processing figure: %s\n', fig_files(k).name);

    % Open the .fig file invisibly ONCE
    try
        fig_handle = openfig(fig_path, 'invisible');
    catch ME
        warning('FIGCONV:OpenError', 'Could not open figure %s: %s', fig_files(k).name, ME.message);
        continue; % Skip to the next figure
    end

    % Get relative path for destination folder structure
    relative_path_from_source = strrep(fig_files(k).folder, source_folder, '');

    % Get base name for output files (e.g., 'Gain Sweep - Hinf - ISO-F')
    [~, name_only, ~] = fileparts(fig_files(k).name);

    % Loop through each desired output format for the current figure
    for i = 1:numel(formats)
        current_format = formats{i};
        
        % Call helper function to print the figure to the current format
        print_figure_to_format(fig_handle, name_only, current_format, ...
                               output_base_folder, relative_path_from_source);
    end
    
    % Close the figure ONCE after all formats are processed for it
    close(fig_handle);
    fprintf('  Finished processing figure: %s\n', fig_files(k).name);
end

disp('All batch conversions complete!');

%% Local Function to Print a Single Figure to a Specific Format
function print_figure_to_format(fig_handle, file_base_name, target_format, ...
                                 output_base_dir, relative_sub_path)
    
    % Define the specific output folder for the current format (e.g., 'figures/png' or 'figures/svg')
    output_format_folder = fullfile(output_base_dir, target_format);

    % Build the corresponding destination directory for this specific figure's path
    dest_folder = fullfile(output_format_folder, relative_sub_path);
    
    % Create destination directory if it doesn't exist
    if ~exist(dest_folder, 'dir')
        mkdir(dest_folder);
    end

    % Set the full output file path
    output_file_name = [file_base_name, '.', target_format];
    output_file_path = fullfile(dest_folder, output_file_name);

    % Determine export options based on target format
    % Assign export_options directly to avoid preallocation warning
    if strcmp(target_format, 'png')
        export_options = {sprintf('-d%s', target_format), '-r300'}; % PNG with 300 DPI resolution
    elseif strcmp(target_format, 'pdf') % Example for PDF, add if 'pdf' is in 'formats'
        export_options = {sprintf('-d%s', target_format), '-bestfit'}; % Best fit for PDF
    else
        export_options = {sprintf('-d%s', target_format)}; % Default for other formats (e.g., SVG, EPS)
    end
    
    % Print the figure
    try
        print(fig_handle, output_file_path, export_options{:});
        fprintf('    Saved %s: %s\n', upper(target_format), output_file_path);
    catch ME
        warning('FIGCONV:PrintError', 'Failed to save %s for %s: %s', upper(target_format), file_base_name, ME.message);
    end
end
