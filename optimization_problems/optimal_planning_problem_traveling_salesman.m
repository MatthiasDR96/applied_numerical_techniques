% Solve the Traveling Salesman problem using a Genetic Algorithm.

% Init 
clear 
clc 
close all

% Create citiy locations
n = 10;
cities = randi(100, n, 2);

% Plot situation
plot(cities(:, 1), cities(:, 2), '*'); hold on;
text(cities(:, 1), cities(:, 2), string(1:n));

% Generate distance matrix
a = meshgrid(1:n);
D = reshape(sqrt(sum((cities(a,:) - cities(a',:)).^2,2)),n,n)

% Init solution
x0 = 2:n;

% Define bounds
lb = repelem(2,n-1,1);
ub = repelem(n,n-1,1);

% Define integer variables
IntCon = (1:1:n-1);

% Define options
options = optimoptions('ga', ...
  'Display','off',...
  'TolCon',1e-6, ...
  'TolFun',1e-6, ...
  'useParallel',true, ...
  'PopulationSize',1000, ...
  'MaxStallGenerations',100, ...
  'StallGenLimit',100, ...
  'MaxGenerations',10000);

% Solve
nvars = n-1;
[x, fval, exitflag, output] = ga(@(x) objective_function(x, cities, D), nvars, [],[],[],[],lb,ub,@(x)nonlcon(x,n),IntCon,options)

% Make tour from solution vector
indices = [1 x 1];
tour = cities(indices,:);

% Plot
plot(tour(:,1), tour(:,2), '-'); hold on;

% Define non-linear constraints
function [c,ceq] = nonlcon(x, n)

    % Equalities
    ceq = []; 
    
    % Inequalities
    c = n - length(unique([1 x 1]));

end

% Define fitness function
function [cost] = objective_function(x, cities, D)
        
    % Make tour from solution vector
    indices = [1 x 1];
    tour = cities(indices,:);
        
    % Calculate cost
    cost = 0;
    for i=1:(length(tour)-1)
        city = tour(i);
        next_city = tour(i+1);
        dist = D(indices(i),indices(i+1));
        cost = cost + dist;
    end
        
end






