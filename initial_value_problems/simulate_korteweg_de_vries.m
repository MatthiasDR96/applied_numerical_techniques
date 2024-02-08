% Init
clear
clc
close all

% Params
T = 10; % Total time 
S = 1000; % Number of time steps
L = 10; % Length of the interval
N = 1000; % Number of spatial discretization points

% Periodic initial conditions
mu = 10;
lambda = 0.1;
x = -L:2*L/N:L;
u_0 = mu * sin((pi*x/L));
% u_0 = lambda./(2 * cosh(sqrt(lambda) * x * 2).^2);

% d1x_ = d1x(u_0,2*L/N)
% figure()
% plot(x, d1x_)
% d2x_ = d2x(u_0,2*L/N)
% figure()
% plot(x, d2x_)


% Plot initial condition
figure();
plot(x, u_0);

% Solve the Korteweg-De Vries equation
[t, x, u] = solve(u_0, L, T, S, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [t, x, u] = solve(u_0, L, T, S, N)

    % Spatial discretization
    h = 2*L/N;
    
    % Spatial vector
    x = -L:h:L;
    
    % Time-stepping
    [t, u] = ode45(@(t,u) F(u, t, h), 0:T/S:T, u_0);

    % Animate
    figure()
    for i=1:S
        plot(x,u(i,:), '.');
        drawnow;
        pause(0.001);
    end
    
end

% Korteweg-de Vries equation
function [F_u] = F(u, ~, h)

    F_u = - d1x(d2x(u, h) + 3*u.^2, h);
    
end

% Function for calculating d/dx
function [d1x] = d1x(y, h)
   
    N=length(y);
    d1x = zeros(N, 1);
    d1x(2:N-1) = (y(3:N) - y(1:N-2)) / 2*h;
    d1x(1) = (y(2) - y(N)) / 2*h;
    d1x(N) = (y(1) - y(N-1)) / 2*h;
    
end

% Function for calculating d^2/dx^2
function d2x = d2x(y, h)

    N=length(y);
    d2x = zeros(N, 1);
    d2x(2:N-1) = (-y(3:N) - 2*y(2:N-1) + y(1:N-2)) / h^2;
    d2x(1) = (-y(2) - 2*y(1) + y(N)) / h^2;
    d2x(N) = (-y(1) - 2*y(N) + y(N-1)) / h^2;
    
end