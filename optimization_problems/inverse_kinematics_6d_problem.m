% The forward kinematics of a robot are given, as well as a target transformation matrix. 
% Calculate the joint values of the robot in order to reach the target destination.

% Clear 
clc
clear
close all

% Init config
q = [0, 0, 0, 0, 0, 0];

% Target destination
T_target = [-sqrt(2)/2, sqrt(2)/2, 0, 0.4; 
            sqrt(2)/2, sqrt(2)/2, 0, -0.4; 
            0, 0, -1, 0.4; 
            0, 0, 0, 1];

% Inverse kinematics
[x, fval, exitflag, output] = fminsearch(@(x) objective_function(x, T_target), q)

% Plot in target state
subplot(2, 2, 1)
plot_robot(q)
plot_frame_t(T_target); 
[T, T01, T12, T23, T34, T45, T56] = forward_kinematics(x);
plot_robot(x); hold on;
plot3(0.4, -0.4, 0.4, 'g.', 'MarkerSize', 20)
plot3(T(1,4), T(2,4), T(3,4), 'r.', 'MarkerSize', 20)
axis equal

% Plot objective axis 1 and 2
subplot(2, 2, 2)
fsurf(@(x, y) objective_function([x y 0.5438 -0.3916 0.1858 0.4031], T_target), [-pi, pi], 'ShowContours', 'on'); hold on;
plot3(x(1), x(2), fval, 'r*')
xlabel('theta1')
ylabel('theta2')
zlabel("Objective")

% Plot objective axis 3 and 4
subplot(2, 2, 3)
fsurf(@(x, y) objective_function([-0.7727 0.8219 x y 0.1858 0.4031], T_target), [-pi, pi], 'ShowContours', 'on'); hold on;
plot3(x(3), x(4), fval, 'r*')
xlabel('theta3')
ylabel('theta4')
zlabel("Objective")

% Plot objective axis 3 and 4
subplot(2, 2, 4)
fsurf(@(x, y) objective_function([-0.7727 0.8219 0.5438 -0.3916 x y], T_target), [-pi, pi], 'ShowContours', 'on'); hold on;
plot3(x(5), x(6), fval, 'r*')
xlabel('theta5')
ylabel('theta6')
zlabel("Objective")

% Plot robot
function plot_robot(q)

    % Get robot kinematics
    [T, T01, T12, T23, T34, T45, T56] = forward_kinematics(q);

    % Plot world frame
    plot_frame_t(eye(4));

    % Plot joint frames and links
    plot_frame_t(T01);
    plot_frame_t(T01*T12*T23);
    plot_frame_t(T01*T12*T23*T34*T45*T56);

    % Labels
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');

    % Axis
    axis equal

end

% Plot frame
function plot_frame_t(t)

    axis_length = 0.2;
    r = t(1:3, 1:3);
    x = t(1,4);
    y = t(2,4);
    z = t(3,4);
    pose_ix = r * [axis_length; 0; 0];
    pose_iy = r * [0; axis_length; 0];
    pose_iz = r * [0; 0; axis_length];
    plot3([x, x + pose_ix(1)], [y, y + pose_ix(2)], [z, z + pose_ix(3)], 'r', linewidth=2); hold on;
    plot3([x, x + pose_iy(1)], [y, y + pose_iy(2)], [z, z + pose_iy(3)], 'g', linewidth=2); hold on;
    plot3([x, x + pose_iz(1)], [y, y + pose_iz(2)], [z, z + pose_iz(3)], 'b', linewidth=2); hold on;

end

% Objective function
function [cost] = objective_function(q, T_target)

    % Forward kinematics
    T = forward_kinematics(q);

    % Position error
    target = T_target(1:3, end);
    squared_distance_to_target = norm(T(1:3, end) - target);

    % Orientation error
    target_orientation = T_target(1:3, 1:3);
    squared_distance_to_orientation = norm(T(1:3, 1:3) - target_orientation);

    % Total error
    cost = squared_distance_to_target + squared_distance_to_orientation;

end

% Forward kinematics
function [T, T01, T12, T23, T34, T45, T56] = forward_kinematics(q)

    % Transformation matrix from frame 0 to frame 1
    T01 = [-cos(q(1)), 0, -sin(q(1)), 0.050 * cos(q(1));
          -sin(q(1)), 0, cos(q(1)), 0.050 * sin(q(1));
          0, 1, 0, 0.457;
          0, 0, 0, 1];
    
    % Transformation matrix from frame 1 to frame 2
    T12 = [-sin(q(2)), -cos(q(2)), 0, -0.440 * sin(q(2));
          cos(q(2)), -sin(q(2)), 0, 0.440 * cos(q(2));
          0, 0, 1, 0;
          0, 0, 0, 1];

    % Transformation matrix from frame 2 to frame 3
    T23 = [-cos(q(3)), 0, -sin(q(3)), 0.035 * cos(q(3));
          -sin(q(3)), 0, cos(q(3)), 0.035 * sin(q(3));
           0, 1, 0, 0;
           0, 0, 0, 1];

    % Transformation matrix from frame 3 to frame 4
    T34 = [-cos(q(4)), 0, -sin(q(4)), 0;
           -sin(q(4)), 0, cos(q(4)), 0;
           0, 1, 0, 0.420;
           0, 0, 0, 1];

    % Transformation matrix from frame 4 to frame 5
    T45 = [-cos(q(5)), 0, -sin(q(5)), 0;
          -sin(q(5)), 0, cos(q(5)), 0;
          0, 1, 0, 0;
          0, 0, 0, 1];

    % Transformation matrix from frame 5 to frame 6
    T56 = [cos(q(6)), -sin(q(6)), 0, 0;
           sin(q(6)), cos(q(6)), 0, 0;
           0, 0, 1, 0;
           0, 0, 0, 1];

    % Transformation matrix from frame 0 to frame 6
    T = T01 * T12 * T23 * T34 * T45 * T56;

end