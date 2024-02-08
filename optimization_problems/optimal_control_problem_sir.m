%% Init

% Clear
clear
clc
close all

%% Uitwerking

% Params
N = 100000; % Population size
tspan = linspace(0, 399, 400);
x0 = [1 - 1/N; 1/N; 0.0; 0.0]; % [s; e; i; r]

% Solve
[t, x_u0] = ode45(@(t, x) seir_model(t, x, 0), tspan, x0);

% Maximale fractie 
i_max = max(x_u0(:,3))

% Bounds
u0 = 0;
lb = 0;
ub = 1;

% Solve
sol = fmincon(@(x) objective_function(x), u0, [], [], [], [], lb, ub, @nonlcon)

% Solve
[t, x_uopt] = ode45(@(t, x) seir_model(t, x, sol), tspan, x0);

%% Plot resultaat

% Subplot voor susceptible en recovered
subplot(2, 1, 1)
plot(t, x_u0(:, 1)); hold on
plot(t, x_u0(:, 4)); hold on
plot(t, x_uopt(:, 1)); hold on
plot(t, x_uopt(:, 4)); hold on
title('Fraction of susceptible and recovered people')
xlabel('Time (days)')
ylabel('Fraction')
legend('Susceptible (u=0)', 'Recovered (u=0)', 'Susceptible (u=opt)', 'Recovered (u=opt)')

% Subplot voor exposed en infected
subplot(2, 1, 2)
plot(t, x_u0(:, 2)); hold on
plot(t, x_u0(:, 3)); hold on
plot(t, x_uopt(:, 2)); hold on
plot(t, x_uopt(:, 3)); hold on
title('Fraction of exposed and infected people')
xlabel('Time (days)')
ylabel('Fraction')
legend('Exposed (u=0)', 'Infected (u=0)', 'Exposed (u=opt)', 'Infected (u=opt)')

% SEIR model
function [dxdt] = seir_model(t, x, u)

    % Params
    alfa = 1 / 5.1; % 1 / incubation time
    gamma = 1 / 3.3; % 1 / infective time
    beta = 2.4 * gamma;

    % Variables
    s = x(1);
    e = x(2);
    i = x(3);
    r = x(4);

    % Differential equations
    dsdt = -(1  - u) * beta * s * i;
    dedt = (1 - u) * beta * s * i - alfa * e;
    didt = alfa * e - gamma * i;
    drdt = gamma * i;

    % Output
    dxdt = [dsdt; dedt; didt; drdt];

end

% Objective function
function [obj] = objective_function(x)
    obj = norm(x);
end

% Nonlinear constraints
function [c, ceq] = nonlcon(u)

    % Params
    N = 100000; % Population size
    tspan = linspace(0, 400, 1000);
    x0 = [1 - 1/N; 1/N; 0.0; 0.0]; % [s; e; i; r]
    
    % Solve
    [~, x] = ode45(@(t, x) seir_model(t, x, u), tspan, x0);

    % Equality constraints
    ceq = [];

    % Inequality constraints
    c = max(x(:, 3)) - 0.02;

end