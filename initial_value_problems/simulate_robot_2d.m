
% Clear
clear
clc
close all

% Init
tspan = [0; 50];
y0 = [0; 0; 0; 0];

% Solve
[t, y] = ode45(@(t, y) odefun(t, y), tspan, y0);

% Plot
figure(1)
plot(t, y(:,1)); hold on; 
plot(t, y(:,2));
xlabel('Time (s)')
ylabel('q (rad)')
legend('q1', 'q2')

% Animate
for i=1:length(t)

    % Plot pendulum
    figure(3);
    plot([0 cos(y(i,1)) cos(y(i,1)) + cos(y(i,1)+y(i,2))], [0 sin(y(i,1)) sin(y(i,1)) + sin(y(i,1) + y(i,2))],'LineWidth',2);
    axis([-2 2 -2 2]);
    grid on;
    
    % Title
    title(['Pendulum state at t=', num2str(round(t(i), 0)), 's']);
    
    % Loop plot
    drawnow;
    
end

function [dy] = odefun(t, y)

    % Params
    m1 = 10;
    m2 = 10;
    l1 = 1;
    l2 = 1;
    g = 9.81;
    c = 10;

    % Get variables
    q1 = y(1);
    q2 = y(2);
    dq1 = y(3);
    dq2 = y(4);

    % Mass matrix
    M = [m1*l1^2 + m2*(l1^2 + 2*l1*l2*cos(q2) + l2^2), m2*(l1*l2*cos(q2)+l2^2); 
         m2*(l1*l2*cos(q2)+l2^2), m2*l2^2];

    % Coriolis matrix
    C = [-m2*l2*l1*sin(q2)*(2*dq1*dq2 + dq2^2); m2*l1*l2*dq1^2*sin(q2)];

    % Gravitational matrix
    G = [(m1 + m2)*l1*g*cos(q1)+m2*g*l2*cos(q1 + q2); m2*g*l2*cos(q1 + q2)];

    % Damping matrix
    D = c.*[dq1; dq2];

    % Acceleration
    dy = zeros(4, 1);
    dy(1:2) = [dq1; dq2];
    dy(3:4) = M \ (- C - G - D);

end