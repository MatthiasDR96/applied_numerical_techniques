% Design a soda can (choose diameter d and height h) to hold a volume of 200 ml, such that the
% manufacturing cost (a function of surface area) is minimized and the constraint h>=2d is obeyed.

% Clear
clear
clc
close all
warning('off','all')
warning

f = @(d, h) (0.5*pi.*d.^2 + pi.*d.*h);
round(f(5, 10))

% Init
x0 = [0; 0];

% Bounds
lb = [0, 0];
ub = [20, 20];

% Linear inequality constraints
A = [2 -1];
b = 0;

% Plot
figure(1)
set(gcf,'position',[0, 0, 1920, 1080])

% Solve
x = fmincon(@(x) objective(x(1), x(2)), x0, A, b, [], [], lb, ub, @(x) nonlcon(x(1), x(2)))

% Objective function
function [cost] = objective(d, h)

    % Plot cilinder
    subplot(1, 3, 3)
    [X,Y,Z] = cylinder(d/2);
    surf(X,Y,Z*h)
    xlim([-10 10])
    ylim([-10 10])
    zlim([0 10])

    % Compute cost
    f = @(d, h) (0.5.*pi.*d.^2 + pi.*d.*h);
    cost = f(d,h)

    % Plot objective curve
    subplot(1, 3, 1)
    fsurf(f, [0, 20]); hold on;
    xlabel('d');
    ylabel('h');
    zlabel('Cost')
    plot3(d, h, cost, '*r')

    % Plot objective contour
    subplot(1, 3, 2)
    [X,Y] = meshgrid(linspace(0, 20), linspace(0, 20));
    contour(X,Y,f(X, Y),'ShowText','on'); hold on;
    plot(linspace(4, 20), 200./(0.25*pi.*linspace(4, 20).^2), 'r');
    plot(linspace(0, 10), 2*linspace(0, 10), 'r');
    plot(d, h, 'r*')
    xlabel('d');
    ylabel('h');
    axis equal

    % Draw
    drawnow()

    % Sleep
    pause(1)

end

% Nonlinear constrains
function [c, ceq] = nonlcon(d, h)
    c = [];
    ceq = 0.25*pi*d.^2.*h - 200;
end