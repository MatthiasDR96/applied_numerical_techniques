% A ball-catching robot has a camera that detects a thrown ball at every 
% second and determines the 3D coordinate of the ball with respect to 
% the robot. There are ten 3D data points obtained from the camera that are 
% given in the 'data.mat' file. Determine the position (x,y) at which 
% the ball hits the ground.

% Clear
clc
clear

% Params
g = 9.81;
v = 100;
theta = 45;
fi = 45;
t = linspace(0, 10, 10);

% Position
x = v*cos(theta)*cos(fi)*t;
y = v*cos(theta)*sin(fi)*t;
z = v*sin(theta)*t - 0.5*g*t.^2;

% Save data
save('data.mat', 'x', 'y', 'z')

% Load data
data = load('data.mat')

% Plot path
figure(1)
plot3(data.x, data.y, data.z, '*'); hold on;
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
grid on;

% Interpolate data
t = linspace(0, 9, 10);
Vx = [ones(size(t))' t'];
cx = Vx\data.x'
Vy = [ones(size(t))' t'];
cy = Vy\data.y'
Vz = [ones(size(t))' t' t.^2'];
cz = Vz\data.z'

% Plot interpolated curve
t = linspace(0, 20, 100);
x = cx(1) + cx(2)*t;
y = cy(1) + cy(2)*t;
z = cz(1) + cz(2)*t + cz(3)*t.^2;
plot3(x, y, z)

% Plot z in time
figure(2)
subplot(3,1,1);
plot(t, x); hold on;
xlabel('Time (s)')
ylabel('x (m)')
grid on;
subplot(3,1,2);
plot(t, y); hold on;
xlabel('Time (s)')
ylabel('y (m)')
grid on;
subplot(3,1,3);
plot(t, z); hold on;
xlabel('Time (s)')
ylabel('z (m)')
grid on;

% Set functions
fx = @(t) cx(1) + cx(2)*t;
fy = @(t) cy(1) + cy(2)*t;
fz = @(t) cz(1) + cz(2)*t + cz(3)*t.^2;

% Calculate root of z-function
r = fzero(fz, 10)

% Evaluate robot location
x_dest = fx(r)
y_dest = fy(r)

% Plot solution
figure(1)
plot3(fx(r), fy(r), fz(r), 'r*')
figure(2)
subplot(3,1,1);
plot(r, fx(r), 'r*')
subplot(3,1,2);
plot(r, fy(r), 'r*')
subplot(3,1,3);
plot(r, fz(r), 'r*')