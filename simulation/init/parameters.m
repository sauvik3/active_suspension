%% ----------------- System Parameters -----------------
m_s = 250;        % Sprung mass (kg)
m_u = 40;         % Unsprung mass (kg)
k_s = 15000;      % Suspension stiffness (N/m)
b_s = 1200;       % Damping coefficient (Ns/m)
k_t = 150000;     % Tire stiffness (N/m)

%% ----------------- Simulation Parameters -----------------
road_len = 1000;              % Length of road to simulate (m)
resolution = 0.1;             % Sampling resolution (m) (if used for spatial road generation)
simulation_speed = 36 / 3.6;  % Simulation speed (m/s)
simulation_time = road_len / simulation_speed; % Time to traverse the road (s)

% Magnitude comparison (each class is 4x rougher than the previous):
% A (1e-6)  <  B (4e-6)  <  C (16e-6)  <  D (64e-6)  <  E (256e-6)  <  F (1024e-6)
class_psd_values = [1e-6, 4e-6, 16e-6, 64e-6, 256e-6, 1024e-6];
iso_classes = {'A', 'B', 'C', 'D', 'E', 'F'};

T = 0:0.01:simulation_time;  % Time vector based on traversal time
x0 = [0; 0; 0; 0];            % Initial state

plot_flag = 1;  % [ 0=No Plot, 1=Plot, 2=Export, 3=Plot+Export ]
