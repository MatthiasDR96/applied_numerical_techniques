% Clear
clc
clear

% Load data
load ('corona_belgium_cut.mat');

% Define solve params
N = 11515793; % Population at start of outbreak 
tspan = 1:length(data.date);
y0 = [11515792, 1, 0]; % [S, I, R]
    
% Optimize for SIR params
best_params = fmincon(@sir_objective, [0.5, 0.5], [], [], [], [], [0 0], [1 1]); 
    
% Disp result
disp(["Best params: ", num2str(best_params)]);

% solve ode
[t,y] = ode45(@(t,y) model(t,y,N,best_params), tspan, y0);

% Extract results
S = y(:,1);
I = y(:,2);
R = y(:,3);

% Plot result
figure(2)
subplot(131)
plot(data.date, S)
subplot(132)
plot(data.date, I, data.date, data.cases)
subplot(133)
plot(data.date, R)

% Ode function
function [dxdt] = model(~, x, N, params)
    
    % Extract state
    S = x(1);
    I = x(2);
    
    % Extract model params
    beta = params(1);
    gamma = params(2);
    
    % Susceptible rate of change
    dsdt = - (beta * I * S / N);
    
    % Infectious rate of change
    didt = (beta * I * S / N) - gamma * I;
    
    % Recovered rate of change
    drdt = gamma * I;
    
    % Derivative of state
    dxdt = [dsdt didt drdt]';
    
end

% Objective function
function [rss] = sir_objective(params)
    
    % Load data
    load('corona_belgium_cut.mat', 'data');

    % Define solve params
    N = 11515793; % Population at start of outbreak 
    tspan = 1:length(data.date);
    y0 = [11515792, 1, 0]; % [S, I, R]
    
    % solve ode
    [t,y] = ode45(@(t,y) model(t,y,N,params), tspan, y0);
    
    % Plot
    plot(t, y(:,2), t, data.cases)
    drawnow
    pause(0.05)

    % RSS Error
    rss = sum((data.cases - y(:,2)).^2);

end

