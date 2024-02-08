% In a manufacturing company, a new CNC machine will arrive. 
% The engineering team wants this machine to be placed in a location in 
% the factory that is closest to all other machines that are in the 
% same manufacturing process as the new CNC machine. Write a function that generates N
% random machine locations (x, y) and that finds the optimal location (x,y)* of the
% CNC machine.

% Init 
clear 
clc
close all

% Params
N = 10;

% Generate N locations
locs = randi(100, N, 2);
 
% Optimize
loc_opt = fminsearch(@(x) objective_function(x, locs), [0, 0]);

% Plot situation
figure(1)
plot(locs(:,1), locs(:,2), 'r.'); hold on;
plot(loc_opt(1), loc_opt(2), '.', 'MarkerSize', 20)

% Plot objective function
figure(2)
fsurf(@(x, y)objective_function([x, y], locs), [0 100], 'ShowContours', 'on')
xlabel('x (m)')
ylabel('y (m)')

% Objective function
function [J] = objective_function(x, locs)
    J = norm(x - locs);
end



