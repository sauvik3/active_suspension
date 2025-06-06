function [t, y] = run_lqr_control(A, B, T, u, x0, gains)
    Q = gains{1};
    R = gains{2};

    % Compute LQR gain
    try
        K = lqr(A, B, Q, R);
    catch ME
        warning(ME.identifier, 'LQR failed: %s', ME.message);
        t = T;
        y = NaN(numel(T), size(A,1));
        return;
    end

    % Interpolate input once to avoid repeated calls
    u_interp = interp1(T, u, T, 'linear', 'extrap');
    if any(isnan(u_interp))
        error('Interpolation returned NaN values!');
    end

    % System dynamics with LQR feedback
    dynamics = @(t, x) A * x + B * (-K * x + interp1(T, u_interp, t, 'linear', 'extrap'));

    % Solve the system
    try
        [t, y] = ode15s(dynamics, T, x0);

        % Check for NaNs or Infs in output
        if any(isnan(y(:))) || any(isinf(y(:)))
            warning('LQR control solution contains NaNs or Infs. Marking output as invalid.');
            y = NaN(size(y));
        end
    catch ME
        warning(ME.identifier, 'ODE solver failed during LQR simulation: %s', ME.message);
        t = T;
        y = NaN(numel(T), size(A,1));
    end
end
