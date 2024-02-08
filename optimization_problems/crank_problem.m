% Problem: A crank mechanism is shown in the figure. L2 = 0.15 m and L3 =
% 0.6m. Link 2 is the input member and equals 60°. Determine theta 3 and s.

% System
% L2*cos(theta2) + L3*cos(theta3)-s = 0
% L2*sin(theta2) + L3*sin(theta3) = 0

% Clear 
clear
clc

% Params
L2 = 0.15; % m
L3 = 0.6; % m
theta2 = 60/180*pi; % radians

% System
f = @(x) [L2*cos(theta2) + L3*cos(x(1)) - x(2);
          L2*sin(theta2) + L3*sin(x(1))];

% Initial guess
x0 = [0; 0;];

% Using fsolve
x = fsolve(f, x0)

% Plot
plot([0, L2*cos(theta2)], [0, L2*sin(theta2)]); hold on;
plot([L2*cos(theta2), L2*cos(theta2) + L3*cos(x(1))], [L2*sin(theta2), L2*sin(theta2)+L3*sin(x(1))]);
plot([0, x(2)], [0, 0]);
axis equal;