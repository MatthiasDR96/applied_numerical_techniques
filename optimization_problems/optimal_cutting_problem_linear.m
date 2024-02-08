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
demand = [4 1 9 10 5 5 3 9 4 1];
%randi(rollLength, n, 1); % Random demand vector

% Min number of rolls needed
numRolls = ceil(sum(demand) / rollLength)

% Params
intcon = 1:(length(demand)*numRolls);
lb = zeros(length(intcon), 1);
ub = ones(length(intcon), 1);

% Constraints
A = [4 0 1 0 9 0 10 0 5 0 5 0 3 0 9 0 4 0 1 0; 
     0 4 0 1 0 9 0 10 0 5 0 5 0 3 0 9 0 4 0 1];
b = repmat(rollLength, numRolls, 1);
Aeq = kron(eye(length(demand)), ones(1, numRolls));
beq = ones(length(demand), 1);

% Solve
cost = repmat(demand, 1, 2)
[sol, fval] = intlinprog(cost, intcon, A ,b, Aeq, beq, lb, ub);

% Reshape
x = reshape(sol, numRolls, length(demand))

% Plot
data = [];
for i=1:numRolls
    tmp = [demand(x(i,:)==1)];
    data = [data;  [tmp zeros(1, length(demand)-length(tmp))]];
end
barh(1:numRolls,data,'stacked'); hold on;
xline(rollLength)
xlabel('Orders')
ylabel('Rolls')
