% Manufacturer produces two products P and Q with machine A and B.
% Machine A takes 50 min to produce P and machine B takes 30 min.
% Machine A takes 24 min to produce Q and machine B takes 33 min.
% In one week, machine A can work 40hrs and machine B 35hrs. 
% The week starts with a stock of 30 units of P and 90 units of Q.
% The week starts with a demand of 75 units of P and 95 units of Q.
% How to plan the production to end the week with the maximum stock?

% Clear
clear
clc

% Define inequality constraints
A = [50 24;
     30 33;
     -1 0;
     0 -1];
b = [40*60; 35*60; -(75 - 30); -(95 - 90)];

% Define equality constraints
Aeq = [];
beq = [];

% Define bounds
lb = [0, 0];
ub = [];

% Objective Z = 6X + 5Y
f = [1; 1];
intcon = [1, 2];

% Solve (Maximize)
[x,fval,exitflag,output] = intlinprog(-f, intcon, A, b, Aeq, beq, lb, ub)

% Plot
x_ = linspace(0, 50);
y_ = linspace(0, 50);
[X,Y] = meshgrid(x_,y_);
Z = f(1)*X + f(2)*Y;

% Plot
contour(X,Y,Z,'ShowText','on'); hold on;
plot(x_, (40*60 - 50*x_)/24, 'b'); % Constraint on Milk
plot(x_, (35*60 - 30*x_)/33, 'k') % Constraint on Choco
plot(x(1), x(2), '*r') % Solution
xlabel('x')
ylabel('y')
xlim([0, 50])
ylim([0, 50])