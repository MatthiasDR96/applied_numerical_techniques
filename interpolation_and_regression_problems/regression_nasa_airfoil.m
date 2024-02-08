% NASA dataset obtained from a series of aerodynamic and acoustic tests of two and three-dimensional airfoil blade sections conducted 
% in an anechoic wind tunnel. The NASA data set comprises different size NACA 0012 airfoils (n0012-il) (see LINK) at various wind 
% tunnel speeds and angles of attack. The span of the airfoil and the observer position were the same in all of the experiments.

%This problem has the following inputs:

%Frequency, in Hertzs.
%Angle of attack, in degrees.
%Chord length, in meters.
%Free-stream velocity, in meters per second.
%Suction side displacement thickness, in meters.
%The only output is:

%Scaled sound pressure level, in decibels.

% Clear
clear
clc
close all

% Read data
data = readtable('AirfoilSelfNoise.csv');

% Plot
subplot(2, 3, 1)
plot(data.f, data.SSPL, '.')
xlabel('f')
subplot(2, 3, 2)
plot(data.alpha, data.SSPL, '.')
xlabel('alpha')
subplot(2, 3, 3)
plot(data.c, data.SSPL, '.'); hold on;
xlabel('c')
subplot(2, 3, 4)
plot(data.U_infinity, data.SSPL, '.')
xlabel('U_infinity')
subplot(2, 3, 5)
plot(data.delta, data.SSPL, '.'); hold on;
xlabel('delta')

% Vandermonde matrix
V = [ones(size(data.f)) data.f data.alpha data.c data.U_infinity data.delta];

% Solve
c = V \ data.SSPL

% Compute Sum of squared errors
SSE = sum((data.SSPL - V*c).^2);

% Compute Total squared error
TSS = sum((data.SSPL - mean(data.SSPL)).^2);

% Compute R^2
r2 = 1 - (SSE/TSS)
