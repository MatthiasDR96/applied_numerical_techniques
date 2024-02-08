% (Assignment in pdf in attachments) Consider the design of a simple tubular symmetric truss. 
% The objective is to optimize the cost of building the column, using variables d, 
% the mean diameter of the column (cm), and t, the thickness of the column (cm).
% P is the compressive load of 2300N. The material used to make the column has a module of elasticity (E) 
% of 650000 and a weight density (ρ) of 0.0020 N/cm3. The column has a yield stress (σy) of 450 N/cm2 and a length (l) of 300 cm.
% Due to available materials the diameter must be ≤ 14.0 cm and ≥ 2.0 cm. Similarly, the thickness must be ≤ 0.8 cm and ≥ 0.2 cm. 
% Safety requires that the induced stress is less than the yield stress and that the induced stress is less than the buckling stress. 
% The cost of the column is equal to the expression 5W + 2d with W being the weight in Newton.

% Clear
clear
clc
close all

% Init
x0 = [0; 0];

% Bounds
lb = [2; 0.2];
ub = [14; 0.8];

% Solve
[sol, fval] = fmincon(@(x) objective_function(x(1), x(2)), x0, [], [], [], [], lb, ub, @(x) nonlcon(x(1), x(2)))

% Plot objective function
fsurf(@(d, t) objective_function(d, t), [lb(1), ub(1), lb(2), ub(2)], 'ShowContours', 'on'); hold on;
xlabel('d (cm)')
ylabel('t (cm)')
plot(sol(1), sol(2), 'r*')
plot3(sol(1), sol(2), objective_function(sol(1), sol(2)), 'r*')
d = linspace(lb(1), ub(1));
plot(d, 2300 ./ (450 * pi * d), 'r') % Indusced stress less than yield stress

% Nonlcon
function [c, ceq] = nonlcon(d, t)

    % Params
    sigma = 450; % kg/cm^2
    P = 2300; % kg
    E = 650000;
    l = 300;

    % Diameter
    do = d + t;
    di = d - t;

    % Induced stress
    induced_stress = P / (pi * d * t);

    % Moment
    I = (pi * (do^4 - di^4)) / 64;

    % Buckling stress
    buckling_stress = (pi^2 * E * I) / (l^2 * pi * d * t);

    % Constraints
    c = [induced_stress - sigma; induced_stress - buckling_stress];
    ceq = [];

end

% Objective function
function [cost] = objective_function(d, t)

    % Params
    rho = 0.0020; % N/cm^3
    l = 300; % cm

    % Diameter
    do = d + t;
    di = d - t;

    % Weight
    W = rho * l * pi *(do^2 - di^2) / 4;

    % Cost
    cost = 5*W + 2*d;

end