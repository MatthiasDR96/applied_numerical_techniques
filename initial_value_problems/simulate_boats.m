% There are four boats at four corners of a square. Each boat starts motoring
% towards the neighboring boat in the clockwise direction (so A pursues B, B pursues C, C pursues
% D and D pursues A). As the boats begin to move, they always move in a direction pointing towards
% the boat they are pursuing, so that they eventually meet in the middle. Each boat motors at a
% constant speed of 1 mile per hour in pursuit of its neighbor. The square has sides of length 1 mile.

% Write down a differential equation that describes the motion of a boat.
% • Sketch the motion of the boats in time.
% • How long until the boats meet in the middle?
% Hint: Try to find any symmetry or convenient coordinates to use that make the problem simpler.
% The simplest solution may surprise you! You might also want to try and draw the situation and
% explain your reasoning.

% Clear
clc
clear
close all

% Params
z0 = [0 0 1 0 1 -1 0 -1]';

% Solve
[t, z] = ode45(@(t,z)dynamics(t,z), [0 1], z0);

% Extract the solution on uniform grid:
x = [z(:,1) z(:,3) z(:,5) z(:,7)];
y = [z(:,2) z(:,4) z(:,6) z(:,8)];

% Plot the solution:
subplot(1,2,1)
plot(x, y); hold on; 
subplot(1,2,2)
plot(t, x)
xlabel('Time (s)')
ylabel('Position (m)')

% Dynamics
function dz = dynamics(~,z)

    % Make system matrix
    A = [-1 0 1 0 0 0 0 0; 
         0 -1 0 1 0 0 0 0; 
         0 0 -1 0 1 0 0 0;
         0 0 0 -1 0 1 0 0;
         0 0 0 0 -1 0 1 0;
         0 0 0 0 0 -1 0 1;
         1 0 0 0 0 0 -1 0;
         0 1 0 0 0 0 0 -1];
    
    % Calculate derivative 
    dz = A*z;

    % Calculate velocities
    v1 = sqrt(dz(1)^2 + dz(2)^2);
    v2 = sqrt(dz(3)^2 + dz(4)^2);
    v3 = sqrt(dz(5)^2 + dz(6)^2);
    v4 = sqrt(dz(7)^2 + dz(8)^2);

    % Normalize velocities
    dz(1) = dz(1)/v1;
    dz(2) = dz(2)/v1;
    dz(3) = dz(3)/v2;
    dz(4) = dz(4)/v2;
    dz(5) = dz(5)/v3;
    dz(6) = dz(6)/v3;
    dz(7) = dz(7)/v4;  
    dz(8) = dz(8)/v4;

end