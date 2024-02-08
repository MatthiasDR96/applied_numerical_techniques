% Simulate a pendulum for 10 seconds with an initial theta of 0.5 radians and initial zero velocity.
% The dynamics of the pendulum is: ddtheta/dt = (g / l) * sin(theta) - c * dtheta;
% Where ddtheta is the angular acceleration, dtheta is the angular velocity, g is the gravitational acceleration of 9.81 m/s^2, 
% l is the length of pendulum of 1 m, and c is the damping constant of 1 Ns/m.

% Clear
clc
clear
close all

% Define solve params
tspan = [0, 10]; % Seconds
x0 = [0.5;  0.0]; % [theta; dtheta]

% Plot vector field
xLim = [-2*pi,2*pi];
yLim = [-7,7];

% State arrays
x = linspace(xLim(1),xLim(2),50); % Theta
y = linspace(yLim(1),yLim(2),50); % Theta dot

% State grid
[xx,yy] = ndgrid(x,y);

% Compute derivative for each state
dxx = zeros(length(x), length(y));
dyy = zeros(length(x), length(y));
for i=1:length(x)
    for j=1:length(y)
        dxdt = model(0, [x(i); y(j); 0; 0]);
        dxx(i, j) = dxdt(1);
        dyy(i, j) = dxdt(2);
    end
end

% Solve ode
[t, x] = ode45(@(t, x) model(t, x), tspan, x0)

% Plot
figure(1); 
quiver(xx,yy,dxx,dyy); hold on;
plot(x(:,1), x(:,2), 'LineWidth', 3)
plot(x(1,1), x(1,2), '.', 'MarkerSize',30)
title('Differential field'); grid on;
xlabel('theta (rad)');
ylabel('dtheta (rad/s)');

% Plot states
figure(2)
subplot(2, 1, 1)
plot(t, x(:,1))
xlabel('Time (s)')
ylabel('Theta (rad)')
subplot(2, 1, 2)
plot(t, x(:,2))
xlabel('Time (s)')
ylabel('Omega (rad/s)')

% Animate
figure(3);
for i=1:length(t)
    
    % Plot pendulum
    plot([0 sin(x(i,1))], [0 cos(x(i,1))],'LineWidth',2);
    axis([-2 2 -2 2]);
    grid on;
    
    % Title
    title(['Pendulum state at t=', num2str(round(t(i), 0)), 's']);
    
    % Loop plot
    drawnow;
    
end

% Dynamic model
function [dxdt] = model(~, x)
    
    % Params
    l = 1;  % length of pendulum (m)
    g = 9.81;  % gravitational acceleration (m/s^2)
    c = 1;  % damping of pendulum (Ns/m)
    
    % Init dxdt
    dxdt = zeros(2, 1);
    
    % Calculate dxdt
	dxdt(1) = x(2);
	dxdt(2) = (g / l) * sin(x(1)) - c * x(2);

end