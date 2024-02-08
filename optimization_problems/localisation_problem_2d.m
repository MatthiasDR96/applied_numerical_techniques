% Determine the xy location of a mobile robot navigating in a space using a LiDAR. 
% There are five reflectors in the room that reflect the emitted light from the LiDAR back to the
% robot. The robot determines the time it takes for the signal to bounce off the reflector and
% return to the LiDAR. Errors on the time signal do not have to be taken into account.

% Generate locations
n = 5; % Number of locations
x = randi([0 100],1,2); % Real location [m]
locations = randi([0 100],n,2); % Location of locations [m]

% Params
v = 300000000; % Speed of light (m/s)

% Calcualte times
tr = 2*(sqrt(((x(1) - locations(:,1)).^2 + (x(2) - locations(:,2)).^2)) / v);

% System
f = @(x) [(x(1) - locations(:,1)).^2 + (x(2) - locations(:,2)).^2 - (v * tr/2).^2];

% Guess
x0 = [0; 0];

% Solve
x_ = fsolve(f, x0)

% Distance to robot x = v*t
radius = v * tr/2;

% Circles
theta = linspace(0, 2*pi)';
cx = cos(theta)*radius' + locations(:,1)';
cy = sin(theta)*radius' + locations(:,2)';

% Plot as surface
%plot(x(1), x(2), 'r*'); hold on;
plot(cx, cy); hold on;
plot(x_(1), x_(2), 'k*'); hold on;
plot(locations(:, 1), locations(:, 2), 'b*')
title('Locations of the reflectors and the robot')
xlim([0 100])
ylim([0 100])


