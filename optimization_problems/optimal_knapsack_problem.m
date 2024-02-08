% There is a container(knapsack) of capacity C = 20. Further more, there is a set 6 of objects. Each object has a weight and a value.
% Determine the number of each item to include in a collection so that the total size is less than or equal to 20 and the total value is as large as possible.

% Number of items
n = 20;  

% Data
maxWeight = 50;
weights = rand(1, n)*maxWeight;
values = rand(1, n)*100;

% Equality constrains
A = weights;
b = maxWeight;

% Objective
f = -values;  
intcon = 1:n;

% Bounds
lb = zeros(n,1);
ub = ones(n,1);

% Solve
[x, fval] = intlinprog(f, intcon, A ,b, [], [], lb, ub) % Positive(fval)

% Result
weight = sum(x'.*weights)
value = sum(x'.*values)