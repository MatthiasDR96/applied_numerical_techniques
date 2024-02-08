% Determine the required damping constant of a pendulum with an initial 
% deflection of 0.5 radians and initial zero velocity to have a maximum deflection of 4 radians.
% The dynamics of the pendulum is: ddtheta/dt = (g / l) * sin(theta) - c * dtheta;
% Where ddtheta is the angular acceleration, dtheta is the angular velocity, g is the gravitational acceleration of 9.81 m/s^2, 
% and l is the length of pendulum of 1 m. Simulate for ten seconds. Use a
% damping constant in the range [0, 10]

% Clear
clc
clear
close all

% Define solve params
tspan = [0, 10]; % Seconds
x0 = [0.5;  0.0]; % [theta; dtheta]

% Solve
sol = fmincon(@objfun, 0, [], [], [], [], 0, 10, @nonlcon)

% Solve ode
[t, x] = ode45(@(t, x) model(t, x, sol), tspan, x0);

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

% Constraint function
function [c, ceq] = nonlcon(decVar)

    % Define solve params
    tspan = [0, 10]; % Seconds
    x0 = [0.5;  0.0]; % [theta; dtheta]
        
    % Solve ode
    [~, x] = ode45(@(t, x) model(t, x, decVar), tspan, x0);

    % Output
    c = max(x(:,1))-4;
    ceq = [];

end

% Objective function
function [cost] = objfun(x)
    cost = x;
end

% Dynamic model
function [dxdt] = model(~, x, c)

    % Params
    l = 1;  % length of pendulum (m)
    g = 9.81;  % gravitational acceleration (m/s^2)
    
    % Init dxdt
    dxdt = zeros(2, 1);
    
    % Calculate dxdt
	dxdt(1) = x(2);
	dxdt(2) = (g / l) * sin(x(1)) - c * x(2);

end

