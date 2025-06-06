function [t, y] = run_smc_control(A, B, T, u, x0, gains)
    % Extract gains from cell array
    K_s = gains{1};
    epsilon = gains{2};

    C_smc = [1 0 0 0];  % Sliding surface: only displacement state

    % Ensure independent initial state
    x0_smc = x0;

    % Pre-check for input interpolation issues
    u_interp = interp1(T, u, T, 'linear', 'extrap');
    if any(isnan(u_interp))
        error('Interpolation returned NaN values!');
    end

    % Define closed-loop system dynamics
    dynamics = @(t, x) A * x + B * (...
        smc_control(C_smc * x(:), K_s, epsilon) + ...
        interp1(T, u, t, 'linear', 'extrap'));

    % Solve using ODE15s
    [t, y] = ode15s(dynamics, T, x0_smc);
end

function u = smc_control(sigma, K_s, epsilon)
    % Smooth saturation for sliding mode control
    sat = @(x) x ./ (1 + abs(x));
    u = -K_s * sat(sigma / epsilon);

    % Apply control saturation
    u = max(min(u, 100), -100);
end
