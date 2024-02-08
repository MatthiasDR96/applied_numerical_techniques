% Clear
clear
clc
close all

% Params
W = 50; % Width of large rectangle
H = 50; % Height of large rectangle
w = 10; % Width of small rectangle
h = 10; % Height of small rectangle
n = 25; % Number of small rectangles

% Max boxes
max_boxes = floor((W*H) / (w*h))

% Linear inequality constraints (The x- and y-coordinates of the boxes cannot
% exceed the width and height of the pallet)
A = [kron(eye(n), [1 0 0]); kron(eye(n), [0 1 0])];
b = [ones(n, 1)*W-w; ones(n, 1)*H-h];

% Bounds
lb = zeros(3*n, 1);
ub = ones(3*n, 1)*(H-h);
ub(3:3:end) = 0; % Orientation is a binary value

% Define options
options = optimoptions('ga', ...
  'Display','off',...
  'TolCon',1e-12, ...
  'TolFun',1e-12, ...
  'FunctionTolerance', 1e-12, ...
  'useParallel',false, ...
  'PopulationSize',1000, ...
  'MaxStallGenerations',1000, ...
  'StallGenLimit',1000, ...
  'MaxGenerations',100000);

% Solve
[x, fval, exitflag, output] = ga(@objective_function, 3*n, A, b, [], [], lb, ub, @nonlcon, 1:3*n, options);

nonlcon(x)

x = reshape(x, 3, length(x)/3)';

% Matrix A
A = [];
for i=1:size(x)
    if x(i,3) == 1
        A = [A; x(i, 1:2) [h w]];
    else
        A = [A; x(i, 1:2) [w h]];
    end
end 

% Plot
for i=1:size(x)
    rectangle('Position', A(i,:), 'FaceColor', rand(1, 3))
end

function [obj] = objective_function(x)

    % Params
    w = 10;
    h = 10;

    % Reshape
    x = reshape(x, 3, length(x)/3)';
   
    % Matrix A
    A = [];
    for i=1:size(x)
        if x(i,3) == 1
            A = [A; x(i, 1:2) [h w]];
        else
            A = [A; x(i, 1:2) [w h]];
        end
    end 

    obj = max(A(:, 2) + A(:, 4));

end

function [c, ceq] = nonlcon(x)

    w = 10;
    h = 10;

    % Reshape
    x = reshape(x, 3, length(x)/3)';
   
    % Matrix A
    A = [];
    for i=1:size(x)
        if x(i,3) == 1
            A = [A; x(i, 1:2) [h w]];
        else
            A = [A; x(i, 1:2) [w h]];
        end
    end 
        
    % Output
    ceq = [];
    c = sum(sum(rectint(A,A) - diag(diag(rectint(A,A)))));

end
