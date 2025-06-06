function [t, y] = run_hinf_control(A, B, T, u, x0, gains)
    Q_hinf = gains{1}; % State weighting matrix
    R_hinf = gains{2}; % Control effort weighting matrix

    try
        % Solve Algebraic Riccati Equation (ARE)
        P = care(A, B, Q_hinf, R_hinf);

        % Compute state-feedback gain K_hinf
        K_hinf = R_hinf \ (B' * P);
    catch ME
        warning(ME.identifier, 'H-Inf failed: %s', ME.message);
        t = T;
        y = NaN(numel(T), size(A,1));
        return;
    end

    % Define closed-loop system dynamics
    dynamics = @(t, x) (A - B * K_hinf) * x + B * interp1(T, u, t, 'linear', 'extrap');

    % Solve the system
    try
        % Solve ODE with H-Infinity feedback
        [t, y] = ode15s(dynamics, T, x0);

        % Check for NaNs or Infs in output
        if any(isnan(y(:))) || any(isinf(y(:)))
            warning('H-Inf control solution contains NaNs or Infs. Marking output as invalid.');
            y = NaN(size(y));
        end
    catch ME
        warning(ME.identifier, 'ODE solver failed during H-Inf simulation: %s', ME.message);
        t = T;
        y = NaN(numel(T), size(A,1));
    end
end
