% Compute the optimal trajectory of a canon ball to reach a given target 
% with minimal energy using Single Shooting. 

% Clear
clear
clc
close all

% Set target
target.x = 6;
target.y = 0;

% Plot
figure(1)
set(gcf,'position',[0, 0, 1920, 1080])

% Plot objective contour
subplot(1, 2, 2)
[X,Y] = meshgrid(linspace(0, 6), linspace(0, 90));
contourf(X,Y,X.^2,'ShowText','on'); hold on;

% Loop
while true

    % Grid search
    x = linspace(4.2, 5, 7);
    y = linspace(pi/10, pi/2-pi/5, 20);
    best_cost = inf;
    best_sol = 0;
    for i=1:length(x)
        for j=1:length(y)
            [c, ceq] = nonLinCst([x(i), y(j)] ,target);
            cost = objective([x(i), y(j)], target);
            if cost < best_cost && abs(ceq(1)) < 0.1
                best_cost = cost;
                best_sol = [x(i), y(j)];
            end
        end
    end

    % Plot solution
    subplot(1, 2, 2)
    plot(best_sol(1), best_sol(2)/pi*180, '.k', 'MarkerSize',50); hold on
    pause(5)
        
    % Clear all
    clf

    % Use FMINCON to solve the constrained optimization problem:
    sol = fmincon(@(decVar)objective(decVar, target), [1, 1], [], [], [], [], [0; 0], [6; 90], @(decVar)nonLinCst(decVar,target));

    % Plot solution
    subplot(1, 2, 2)
    plot(sol(1), sol(2)/pi*180, '.k', 'MarkerSize',50); hold on
    pause(5)

    % Clear all
    clf

end

function [y] = simulate(decVar)

    % Unpack decision variables
    v0 = decVar(1); 
    th0 = decVar(2);

    % Set up initial conditions 
    x0 = 0;  
    y0 = 0;
    dx0 = v0*cos(th0);
    dy0 = v0*sin(th0);
    
    % Params
    userFun = @(t,z)cannon_dynamics(t,z);
    tSpan = [0,100];
    z0 = [x0;y0;dx0;dy0];
    
    % Event options
    options = odeset('Events',@groundEvent,'Vectorized','on');
    
    % Solve
    [~, y] = ode45(userFun, tSpan, z0, options);

end

% Non-linear constraint function for multiple shooting
function [C, Ceq] = nonLinCst(decVar,target)

    % Simulate
    y = simulate(decVar);

    % End state
    xFinal = y(end, 1);
    yFinal = y(end, 2);

    % Define constraints
    C = []; % No inequality constraints
    Ceq = [xFinal - target.x; yFinal - target.y];  % Boundary Condition

    % Plot solution
    subplot(1, 2, 1)
    plot(y(:, 1),y(:, 2),'LineWidth',3); hold on;
    plot(y(end,1), y(end,2), '.r', 'MarkerSize',30)
    plot(6, 0, '.g', 'MarkerSize',30)
    title("Canon simulation")
    xlabel('X  Position (m)');
    ylabel('Y  Position (m)');
    axis([0 8 0 4])
    drawnow()
    
    % Plot constraint
    %subplot(2, 2, 3)
    %error = xFinal - target.x;
    %if abs(error) < 0.1
        %plot(decVar(1), decVar(2)/pi*180, '.g', 'MarkerSize',30); hold on
    %else
        %plot(decVar(1), decVar(2)/pi*180, '.r', 'MarkerSize',30); hold on
    %end
    %title('Constraint function')
    %xlabel('Velocity (m/s)')
    %ylabel('Angle (deg)')
    %axis([0 6 0 90])
    %drawnow()

end

% Objective function for optimization 
function cost = objective(decVar, target)
    
    % Simulate
    y = simulate(decVar);

    % Initial energy of the cannon ball (v0^2)
    cost = decVar(1)^2;

    % Plot objective contour
    subplot(1, 2, 2)
    if abs(y(end, 1) - target.x) < 0.1
        plot(decVar(1), decVar(2)/pi*180, '.g', 'MarkerSize',30); hold on
    else
        plot(decVar(1), decVar(2)/pi*180, '.r', 'MarkerSize',30); hold on
    end
    title('Objective function contour')
    xlabel('Velocity (m/s)')
    ylabel('Angle (deg)')
    axis([0, 6, 0, 90])
    drawnow()

    % Plot objective surface
    %subplot(2, 2, 4)
    %[X,Y] = meshgrid(linspace(0, 6), linspace(0, 90));
    %surf(X,Y,X.^2); hold on;
    %if abs(y(end, 1) - target.x) < 0.1
        %plot3(decVar(1), decVar(2)/pi*180, cost, '.g', 'MarkerSize',30); hold on
    %else
        %plot3(decVar(1), decVar(2)/pi*180, cost, '.r', 'MarkerSize',30); hold on
    %end
    %title('Objective function surface')
    %xlabel('Velocity (m/s)')
    %ylabel('Angle (deg)')
    %axis([0, 6, 0, 90])
    %drawnow()

end

% Dynamics
function [dz] = cannon_dynamics(~,z)

    % Dynamic params
    m = 1; % Mass [kg]
    g = 1; % Gravity constant [N/kg]
    c = 0.2; % Quadratic drag coefficient [Ns/m]

    % Init state
    dz = zeros(size(z));
    
    % First-order form (derivative of position states is velocity states)
    dz(1:2,:) = z(3:4,:);
    
    % Compute the speed (used for drag force calculation)
    dx = z(3,:);
    dy = z(4,:);
    v = sqrt(dx.*dx + dy.*dy); 
    
    % Record the accelerations (derivative of velocity states)
    dz(3,:) = -m*c*dx.*v;
    dz(4,:) = -m*c*dy.*v - m*g; 

end

% Ground event
function [value, isterminal, direction] = groundEvent(~,z)

    % Vertical position of the cannon ball
    height = z(2,:);  
    
    % The value that we want to be zero
    value = height; 

    % Halt integration
    isterminal = true; 

    % Locate zeros where the event function is decreasing
    direction = -1; 

end







