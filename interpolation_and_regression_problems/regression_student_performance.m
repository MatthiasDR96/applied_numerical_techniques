% Clear
clear
clc
close all

% Read data
data = readtable('Student_Performance.csv')

%linspace(0, size(data.PerformanceIndex)[1])
%plot(data.PerformanceIndex, linspace(0, size(data.PerformanceIndex)[1]))

% Plot
subplot(2, 3, 1)
plot(data.HoursStudied, data.PerformanceIndex, '.')
xlabel('HoursStudied')
subplot(2, 3, 2)
plot(data.PreviousScores, data.PerformanceIndex, '.')
xlabel('PreviousScores')
subplot(2, 3, 4)
plot(data.SleepHours, data.PerformanceIndex, '.')
xlabel('SleepHours')
subplot(2, 3, 5)
plot(data.SampleQuestionPapersPracticed, data.PerformanceIndex, '.'); hold on;
xlabel('SampleQuestionPapersPracticed')

% Vandermonde matrix
V = [ones(size(data.HoursStudied)) data.HoursStudied data.PreviousScores data.SleepHours data.SampleQuestionPapersPracticed];

% Solve
c = V \ data.PerformanceIndex

% Compute Sum of squared errors
SSE = sum((data.PerformanceIndex - V*c).^2);

% Compute Total squared error
TSS = sum((data.PerformanceIndex - mean(data.PerformanceIndex)).^2);

% Compute R^2
r2 = 1 - (SSE/TSS)
