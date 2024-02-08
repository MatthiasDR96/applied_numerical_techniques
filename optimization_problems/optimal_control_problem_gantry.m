% Clear
clear
clc
close all

% Simulation time
T = 5; 

% Initial condition
x0 = [4.2666; -3.2507; 0.7099; -0.0461];

% Linear equality constraint
Aeq = [5 5^2 5^3 5^4];
beq = 0;

% Polynomial
p = @(t, x) (x(1)*t + x(2)*t.^2 + x(3)*t.^3 + x(4)*t.^4);

% Optimize control signal
options= optimoptions('fmincon','EnableFeasibility',true, 'SubproblemAlgorithm', 'cg') 
[sol, fval] = fmincon(@(x) objective(x, p), x0, [], [], Aeq, beq, [], [], @(x) nonlcon(x, p), options)

% Simulate solution
[t, x] = ode45(@(t, x) model(t, x, p(t, sol)), [0 T], [0; 0; 0; 0]);

[c, ceq] = nonlcon(sol, p)

% Animate
figure(3);
for i=1:length(t)
    
    % Plot pendulum
    plot([x(i,1), x(i,1) + sin(x(i,3))], [0 -cos(x(i,3))],'LineWidth',2);
    axis([-2 10 -2 2]);
    grid on;
    
    % Title
    title(['Pendulum state at t=', num2str(round(t(i), 0)), 's']);
    
    % Loop plot
    drawnow;
    
end

% Plot
t = linspace(0, 5);
plot(t, p(t, sol));
xlabel('Time (s)')
ylabel('u (N)')

function [c, ceq] = nonlcon(x, p)

    % Solve 
    [~, y] = ode45(@(t, y) model(t, y, p(t, x)), [0 5], [0; 0; 0; 0]);

    % Constraints
    ceq = y(end, :) - [5 0 0 0];
    c = [];

end

function obj = objective(x, p)

    % Solve
    [~, y] = ode45(@(t, y) model(t, y, p(t, x)), [0 5], [0; 0; 0; 0]);

    % Objective
    obj = sum(y(:,3).^2);

end   


function dx = model(~, x, u)

    % Params
    g = 9.81;
    m = 0.23;
    M = 1.07;
    L = 0.33;

    % System matrix
    A = [0 1 0 0;
         0 0 g*m/M 0;
         0 0 0 1;
         0 0 -g*(M+m)/(L*M) 0];

    % Control matrix
    B = [0; 1/M; 0; -1/(L*M)];

    % Dynamics 
    dx = A*x + B.*u;

end