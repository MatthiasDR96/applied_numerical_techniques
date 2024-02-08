% A planar robot with two rotational degrees of freedom can be moved using
% its joints q=[theta1; theta2]. The robot needs to reach a target at
% Cartesian position X = [1.5; 0.5]. The Forward Kinematics of the robot
% that computes the Cartesian position of the end-effector given the
% joints is given in the function 'fk'. This function takes the joint
% values q=[theta1; theta2] as input and returns the Cartesian position of
% the end-effector X = [x, y]; Find the joint angles in order for the robot
% to reach the target location. 

% Clear
clear
clc
close all

% Bounds (elbow up)
lb1 = [-pi; -pi/2];
ub1 = [pi; 0];

% Bounds (elbow down)
lb2 = [-pi; 0];
ub2 = [pi; pi/2];

% Target
target = [1.5; 0.5];

% Solve
[x, fval, exitflag, output] = fmincon(@(x) objective(x, target), [0; 0], [], [], [], [], lb2, ub2)

% Plot objective
subplot(1, 2, 1)
fsurf(@(x, y) objective([x; y], target), [-pi, pi], 'ShowContours', 'on'); hold on;
plot3(x(1), x(2), fval, 'r*')
xlabel('theta1')
ylabel('theta2')
zlabel("Objective")

% Plot
subplot(1, 2, 2)
x_coor = [0 cos(x(1)) cos(x(1)) + cos((x(1) + x(2)))];
y_coor = [0 sin(x(1)) sin(x(1)) + sin((x(1) + x(2)))];
plot(x_coor, y_coor, '.', 'MarkerSize', 20); hold on;
plot(x_coor, y_coor, '-')
plot(target(1), target(2), 'r*')
xlabel('X Position (m)')
ylabel('Y Position (m)')
axis([-2, 2, -2, 2])
axis equal

% Objective function
function [cost] = objective(x, target)

    % Forward kinematics
    [x, y] = fk(x);

    % Error
    cost = (x - target(1))^2 + (y - target(2))^2;

end

% Forward kinematics
function [x, y] = fk(q)

    % Params
    l1 = 1;
    l2 = 1;

    % Forward kinematics
    x = l1*cos(q(1)) + l2*cos(q(1) + q(2));
    y = l1*sin(q(1)) + l2*sin(q(1) + q(2));

end