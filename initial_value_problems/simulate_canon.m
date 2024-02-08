% Simulate a canonball launched at an angle of 45° with an initial velocity of 10 m/s
% The canonball experiences a drag force with a quadratic drag coefficient c of 0.2 

% Clear
clc
clear
close all

% Params
v0 = 10;
th0 = 45*(pi/180);
tspan = [0,7];
z0 = [0;0;v0*cos(th0);v0*sin(th0)];

% Event options
options = odeset('Events',@groundEvent,'Vectorized','on');

% Solve
[t, z] = ode45(@(t,z)cannon_dynamics(t,z), tspan, z0, options);

% Extract the solution on uniform grid:
x = z(:,1); 
y = z(:,2); 

% Plot the solution:
plot(x(1),y(1),'b.','MarkerSize',35); hold on;  % Start
plot(x(end),y(end),'rx','LineWidth',4,'MarkerSize',14)   % Finish
plot(x,y,'k-','LineWidth',3);  % Trajectory
axis equal;
xlabel('x (m)')
ylabel('y (m)')
title('Cannon Ball Trajectory')

% Dynamics
function dz = cannon_dynamics(~,z)

    % Quadratic drag coefficient:
    c = 0.2;

    % Init state
    dz = zeros(size(z));
    
    % First-order form (derivative of position states is velocity states)
    dz(1:2,:) = z(3:4,:);
    
    % Compute the speed (used for drag force calculation)
    dx = z(3,:);
    dy = z(4,:);
    v = sqrt(dx.*dx + dy.*dy); 
    
    % Compute the force vectors
    fx = -c*dx.*v;
    fy = -c*dy.*v - 1;  % Assume unit gravity
    
    % Record the accelerations (derivative of velocity states)
    dz(3,:) = fx;  % Assume unit point-mass
    dz(4,:) = fy;  % Assume unit point-mass

end

% Ground event
function [value, isterminal, direction] = groundEvent(~,z)

    % Vertical position of the cannon ball
    height = z(2,:);  
    
    % The value that we want to be zero
    value = height; 

    % Halt integration
    isterminal = true; 

    % Locate zeros where the event function is decreasing
    direction = -1; 

end

