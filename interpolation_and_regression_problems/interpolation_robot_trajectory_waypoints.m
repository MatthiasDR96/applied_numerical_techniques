% An industrial robot arm wants to move in 1 dimension through three points. 
% It wants to do this using a cubic polynomial in the form of 
% s(t) = a0*t^0 + a1*t^1 + a2*t^2 + a3*t^3
% where s denotes the position and t denotes the time. 
% The robot should move through positions s1=0, s2=2, and s3=3 at times t1=0, t2=3 and t2=6.
% The trajectory should start and end with zero velocity but the velocity at position s2 should be 2m/s. 
% Determine the coefficients of the trajectory polynomial and plot s and 
% its first and second derivatives on a graph with labeled axes. 

% Equations
% s1(t) = a10*t^0 + a11*t^1 + a12*t^2 + a13*t^3
% ds1(t) = a11*t^0 + 2*a12*t^1 + 3*a13*t^2
% s2(t) = a20*t^0 + a21*t^1 + a22*t^2 + a23*t^3
% ds2(t) = a21*t^0 + 2*a22*t^1 + 3*a23*t^2

% Constraints
% s1(0) = a10*0^0 + a11*0^1 + a12*0^2 + a13*0^3 = 0
% s1(3) = a10*3^0 + a11*3^1 + a12*3^2 + a13*3^3 = 2
% ds1(0) = a11*0^0 + 2*a12*0^1 + 3*a13*0^2 = 0
% ds1(3) = a11*3^0 + 2*a12*3^1 + 3*a13*3^2 = 2
% s2(3) = a20*3^0 + a21*3^1 + a22*3^2 + a23*3^3 = 2
% s2(6) = a20*6^0 + a21*6^1 + a22*6^2 + a23*6^3 = 3
% ds2(3) = a21*3^0 + 2*a22*3^1 + 3*a23*3^2 = 2
% ds2(6) = a21*6^0 + 2*a22*6^1 + 3*a23*6^2 = 0

% Init
clear
clc

% Coefficient matrix
A = [[1 0 0 0 0 0 0 0]; [1 3 3^2 3^3 0 0 0 0]; [0 1 0 0 0 0 0 0]; [0 1 6 3*3^2 0 0 0 0]; [0 0 0 0 1 3 3^2 3^3]; [0 0 0 0 1 6 6^2 6^3]; [0 0 0 0 0 1 2*3 3*3^2]; [0 0 0 0 0 1 2*6 3*6^2]]

% Known vector
b = [0 2 0 2 2 3 2 0]';

% Solution
x = A\b;

% Evaluate solution of segment 1
t1 = linspace(0,3,100);
s1 = x(1)*t1.^0 + x(2)*t1.^1 + x(3)*t1.^2 + x(4)*t1.^3;
ds1 = x(2)*t1.^0 + 2*x(3)*t1.^1 + 3*x(4)*t1.^2;
dds1 = 2*x(3)*t1.^0 + 6*x(4)*t1.^1;

% Evaluate solution of segment 2
t2 = linspace(3,6,100);
s2 = x(5)*t2.^0 + x(6)*t2.^1 + x(7)*t2.^2 + x(8)*t2.^3;
ds2 = x(6)*t2.^0 + 2*x(7)*t2.^1 + 3*x(8)*t2.^2;
dds2 = 2*x(7)*t2.^0 + 6*x(8)*t2.^1;

% Plot solution
figure(2)
subplot(1, 3, 1)
plot(t1, s1); hold on;
plot(t2, s2)
title("s(t)")
xlabel("t (s)")
subplot(1, 3, 2)
plot(t1, ds1); hold on;
plot(t2, ds2)
title("ds(t)")
xlabel("t (s)")
subplot(1, 3, 3)
plot(t1, dds1); hold on;
plot(t2, dds2)
title("dds(t)")
xlabel("t (s)")

