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
z0 = [1 1 0 0]';


% Solve
[t, z] = ode45(@(t,z)dynamics(t,z), [0 100], z0);

% Extract the solution on uniform grid:
x = z(:,1);
y = z(:,2);

% Plot the solution:
plot(t, x); hold on; 
plot(t, y)
xlabel('Time (s)')
ylabel('Position (m)')

% Dynamics
function dz = dynamics(~,z)

    w1 = 1;
    w2 = 1.5;
    e = 0.5;

    % Make system matrix
    A = [0 0 1 0; 
         0 0 0 1; 
         -w1^2-e e 0 0;
         e -w2^2-e 0 0];
    eig(A)
    
    % Calculate derivative 
    dz = A*z;

end