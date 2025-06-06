%% ----------------- CRG File Processor -----------------
function [road_length, resolution, road_profile] = process_crg_file(crg_file, T, speed)
    data = crg_read(crg_file);
    data = crg_check(data);  % Check for validity

    % Convert time vector to position (s)
    s = speed * T;
    s = min(max(s, data.head.ubeg), data.head.uend);  % Clamp to valid domain
    v = zeros(size(s));  % Evaluate along centerline

    z = crg_eval_uv2z(data, [s(:), v(:)]);  % Evaluate road height profile

    road_profile = z(:);  % Column vector
    resolution = mean(diff(s));
    road_length = data.head.uend - data.head.ubeg;

    % CRG Information
    fprintf('CRG Length: %.2f m, Resolution: %.4f m\n', road_length, resolution);
end
