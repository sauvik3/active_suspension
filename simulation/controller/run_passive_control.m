function [t, y] = run_passive_control(A, B, T, u, x0)
    % Apply a moving average filter to smooth the road profile
    u_filtered = movmean(u, 10);  % Adjust window size if needed

    % Define the passive dynamics
    dynamics_passive = @(t, x) A * x + B * interp1(T, u_filtered, t, 'linear', 'extrap');

    % Solve the system using ode15s
    [t, y] = ode15s(dynamics_passive, T, x0);
end
