% Problem: obtain the angle with respect to the horizontal at which a canon
% needs to be pointed at if a canonball needs to hit the ground after 1 second at
% an initial velocity of 10 m/s.

% Clear
clc
clear

% Params
t = 1;
g = 9.81;
v = 10; % m/s

% Function
f = @(x) (v*sin(x/180*pi)*t - 0.5*g*t^2);

% Initial guess
x0 = 1;

% Solve
theta = fzero(f, x0)

% Evaluate
f(theta)

% Plot
t = linspace(0, 1, 100);
x = v*cos(theta/180*pi)*t;
y = v*sin(theta/180*pi)*t - 0.5*g*t.^2;
plot(x, y)
xlabel('x')
ylabel('y')
