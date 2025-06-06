function u = generate_road_profile(road_len, resolution, T, class_psd)
    n = (0:road_len/resolution-1) / road_len;
    psd = class_psd * (n / 0.1) .^ -2;
    psd(1) = 0; % Remove DC offset
    
    % Generate random phase and inverse FFT
    random_phase = exp(1j * 2 * pi * rand(size(n)));
    road_profile = ifft(sqrt(psd) .* random_phase, 'symmetric');
    road_profile = road_profile * sqrt(road_len/resolution);
    
    % Interpolate to match time vector
    u = interp1(linspace(0, road_len, road_len/resolution), road_profile, T, 'linear', 'extrap');
end
