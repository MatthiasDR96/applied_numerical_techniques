% An industrial robot arm wants to move in 1 dimension between two points. 
% It wants to do this using a quintic polynomial in the form of 
% s(t) = a0*t^0 + a1*t^1 + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 
% where s denotes the position and t denotes the time. 
% The robot should move between position s=0 and s=1.
% The trajectory should take 5 seconds in total and should start and end 
% with zero velocity and zero acceleration. 
% Determine the coefficients of the trajectory polynomial and plot s and 
% its first and second derivatives on a graph with labeled axes. 

% Equations
% s(t) = a0*t^0 + a1*t^1 + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 
% ds(t) = a1*t^0 + 2*a2*t^1 + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4 
% dds(t) = 2*a2*t^0 + 6*a3*t^1 + 12*a4*t^2 + 20*a5*t^3 

% Constraints
% s(0) = a0 = 0
% s(5) = a0 + a1*5 + a2*5^2 + a3*5^3 + a4*5^4 + a5*5^5 = 1
% ds(0) = a1 = 0
% ds(5) = a1 + 2*a2*5 + 3*a3*5^2 + 4*a4*5^3 + 5*a5*5^4 = 0
% dds(0) = 2*a2 = 0
% dds(5) = 2*a2 + 6*a3*5^1 + 12*a4*5^2 + 20*a5*5^3  = 0

% Coefficient matrix
A = [[1 0 0 0 0 0]; [1 5 5^2 5^3 5^4 5^5]; [0 1 0 0 0 0]; [0 1 10 3*5^2 4*5^3 5*5^4]; [0 0 2 0 0 0]; [0 0 2 6*5 12*5^2 20*5^3]];

% Known vector
b = [0 1 0 0 0 0]';

% Solution
x = A\b;

% Evaluate solution
t = linspace(0,5, 100);
s = x(1)*t.^0 + x(2)*t.^1 + x(3)*t.^2 + x(4)*t.^3 + x(5)*t.^4 + x(6)*t.^5;
ds = x(2)*t.^0 + 2*x(3)*t.^1 + 3*x(4)*t.^2 + 4*x(5)*t.^3 + 5*x(6)*t.^4;
dds = 2*x(3)*t.^0 + 6*x(4)*t.^1 + 12*x(5)*t.^2 + 20*x(6)*t.^3;

% Plot solution
subplot(1, 3, 1)
plot(t, s)
title("s(t)")
xlabel("t (s)")
subplot(1, 3, 2)
plot(t, ds)
title("ds(t)")
xlabel("t (s)")
subplot(1, 3, 3)
plot(t, dds)
title("dds(t)")
xlabel("t (s)")

