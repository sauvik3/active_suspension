function [t, y] = run_pid_control(A, B, T, u, x0, gains)
    % Extract the PID gains from the cell array
    Kp = gains{1};  % Proportional gain
    Ki = gains{2};  % Integral gain
    Kd = gains{3};  % Derivative gain

    % Define system dynamics with PID control
    dynamics = @(t, x) [
        A * x(1:end-2) + B * (pid_control(x(end-1), x(end), Kp, Ki, Kd) + interp1(T, u, t, 'linear', 'extrap'));
        x(end-1);  % Integral term
        x(end-2)   % Previous error
    ];
    
    % Integrate the system
    [t, y] = ode15s(dynamics, T, [x0; 0; 0]); % Append integral and previous error states
end

function u = pid_control(error, integral, Kp, Ki, Kd)
    % Controller limits
    u_max = 100;  % Upper limit of control output
    u_min = -100; % Lower limit of control output
    k_aw = 0.1;   % Anti-windup gain

    % Compute PID terms
    u = Kp * error + Ki * integral + Kd * (error - integral);

    % Apply saturation limits
    if u > u_max
        u = u_max;
        integral = integral - k_aw * (u - u_max); % Anti-windup correction
    elseif u < u_min
        u = u_min;
        integral = integral - k_aw * (u - u_min); % Anti-windup correction
    end
end
