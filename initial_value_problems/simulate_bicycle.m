% Clear
clc
clear
close all

% Define solve params
tspan = [0 10];
x0 = [0;0;0;0]; % [x, y, theta, v]

% Solve ode
[t, x] = ode45(@(t,x) odefun(t,x), tspan, x0);

% Animate
figure(1)
for i=1:length(t)

    % Plot robot
    plot_robot([x(i,1), x(i,2), x(i,3)]); hold on;
    plot(x(i,1), x(i,2), '.b');
    axis([-2 2 -2 2]);
    grid on;

    % Title
    title(['Robot state at t=', num2str(round(t(i), 0)), 's']);

    % Loop plot
    drawnow;
    pause(0.1);

end

% Ode function
function [dxdt] = odefun(~, x)
    
    % Get control signal
    u = 1;
    
    % Params
    lr = 0.5;
    lf = 0.5;
    L = lr+lf;
    min_steer = -pi/4;
    max_steer = pi/4;

    % Control signals
    ax = 1;
    delta = u;
    
    % Limit steering angles
    if delta >= max_steer
        delta = max_steer;
    end
    if delta <= min_steer
        delta = min_steer;
    end
    
    % Kinematic model
    dxdt = zeros(4, 1);
    dxdt(1) = x(4) * cos(x(3));
    dxdt(2) = x(4) * sin(x(3));
    dxdt(3) = (x(4) / L) * tan(delta); 
    dxdt(4) = ax;
    
end

function [] = plot_robot(state)
    
    % Vehicle coordinates
    vehicle_points = [0.5 0 1; -0.5 0 1]';

    % Plot center
    plot(state(1), state(2), 'b.', 'MarkerSize', 10);
    
    % Convert robot point to new pos
    T = [cos(state(3)) -sin(state(3)) state(1); sin(state(3)) cos(state(3)) state(2); 0 0 1];
    robot_pos = T * vehicle_points;
    
    % Plot chassis
    plot(robot_pos(1, :), robot_pos(2, :), '.b-', 'MarkerSize', 10);
    
end

