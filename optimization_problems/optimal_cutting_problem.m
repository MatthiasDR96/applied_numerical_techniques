% A company manufactures polymer rolls of length 30m and cuts them according to customized order lengths.
% Given a list of customer orders, determine the minimal number of rolls of 30m that are needed, 
% and for those rolls, determine the optimal cutting patterns while reducing the waste. 
% Approach the problem as an assignment problem where each of the orders needs to be assigned to a roll. 
% The total length of the orders assigned to a certain roll can of course not exceed its total length. 

% Clear
clear 
clc
close all   

% Define the problem parameters
n = 10; % Number of orders
rollLength = 50; % Roll length
demand = randi(rollLength, n, 1); % Random demand vector

% Min number of rolls needed
numRolls = ceil(sum(demand) / rollLength)

% Bounds
lb = ones(size(demand));
ub = repmat(numRolls, size(demand));

% Functions
f = @(x) objective_function(x, demand, rollLength);
nonlcon = @(x) constraints_function(x, demand, rollLength);

% Params
nvars = length(demand);
intcon = 1:nvars;

% Define options
options = optimoptions('ga', ...
  'Display','off',...
  'TolCon',1e-6, ...
  'TolFun',1e-10, ...
  'useParallel',false, ...
  'PopulationSize',100, ...
  'MaxStallGenerations',1000, ...
  'StallGenLimit',1000, ...
  'MaxGenerations',10000);

% Solve
[sol, fval, exitflag, output] = ga(f, nvars, [], [], [], [], lb, ub, nonlcon, intcon, options)

% Result
waste = objective_function(sol, demand, rollLength)
cons = constraints_function(sol, demand, rollLength)

% Plot
x = intcon;
data = [];
for i=1:numRolls
    tmp = [demand(sol==i)]';
    data = [data;  [tmp zeros(1, nvars-length(tmp))]];
end
barh(1:numRolls,data,'stacked'); hold on;
xline(rollLength)
xlabel('Orders')
ylabel('Rolls')

% Objective function
function [total_waste] = objective_function(x, demand, rollLength)

    % Number of different demand lengths
    numRolls = ceil(sum(demand) / rollLength); % Min number of rolls needed

    % Calculate waste
    total_waste = 0;
    for i=1:numRolls
        waste = max(0, rollLength - sum(demand(x==i)));
        total_waste = total_waste + waste;
    end

end

% Nonlinear constraints
function [c, ceq] = constraints_function(x, demand, rollLength)

    % Number of different demand lengths
    numRolls = ceil(sum(demand) / rollLength); % Min number of rolls needed

    % Calculate waste
    c = [];
    ceq = [];
    for i=1:numRolls
        shortage = rollLength - sum(demand(x==i));
        c = [c  -shortage];
    end

end

