% Compute the optimal trajectory of a canon ball to reach a given target 
% with minimal energy using Multiple Shooting. 

% Clear
clc
clear

% Params
nState = 4;
nGrid = 100;

% Target
start = [0; pi; 0; 0];
target = [1; 0; 0; 0];

%% Initial guess
t = linspace(0, 10, nGrid);
y = interp1([0, 10], [start, target]', t);

% Plot solution
figure(1)
subplot(4, 1, 1)
plot(t,y(:, 1), '*'); 
subplot(4, 1, 2)
plot(t,y(:, 2), '*'); 
subplot(4, 1, 3)
plot(t,y(:, 3), '*');  
subplot(4, 1, 4)
plot(t,y(:, 4), '*'); 

%% Multiple-shooting

% Store the initial guess for the problem
problem.x0 = [[0, t(end)], reshape(y,1,nState*nGrid), zeros(1,nGrid)];
problem.lb = []; % Lower bound on decision variables
problem.ub = []; % Upper bound on decision variables

% Set up the linear constraints (there are none);
problem.Aineq = [];  problem.Aeq = [];
problem.bineq = [];  problem.beq = [];

% Set up the user-defined functions:
problem.objective = @(decVar)objective(decVar(nState*nGrid+2:end));  % Objective (cost) function
problem.nonlcon = @(decVar)nonLinCst(decVar, target);   % NonLinear constraints

% Set up the options for the solver:
problem.solver = 'fmincon';
problem.options = optimset('MaxFunEvals',1e4,'MaxIter',100,'Display','off');

% Use FMINCON to solve the constrained optimization problem:
[zSol, fVal, exitFlag] = fmincon(problem);

% Call the constraint function one final time to get the trajectory:
[~, ~, t, y] = nonLinCst(zSol,target)

%% Animate
figure(2);
for i=1:length(t)
    
    % Plot pendulum
    %plot(y(:,1)+sin(y(:,2)),cos(y(:,2)),'LineWidth',2); hold on;
    plot([y(i,1), y(i,1)+sin(y(i,2))], [0, cos(y(i,2))],'LineWidth',2);
    axis equal;
    grid on;
    axis([-2 2 -2 2]);
    hold off;
    
    % Title
    title(['Pendulum state at t=', num2str(round(t(i), 0)), 's']);
    
    % Loop plot
    drawnow;
    
end

% Plot trajectory in time
figure(4)
subplot(4, 1, 1)
plot(t,y(:, 1)); 
ylabel('X');
xlabel('Time (s)');
subplot(4, 1, 2)
plot(t,y(:, 2)); 
ylabel('Y');
xlabel('Time (s)');
subplot(4, 1, 3)
plot(t,y(:, 3)); 
ylabel('Vx');
xlabel('Time (s)');
subplot(4, 1, 4)
plot(t,y(:, 4)); 
ylabel('Vy');
xlabel('Time (s)');

% Nonlinear constraints function
function [C, Ceq, tSol,zSol] = nonLinCst(z, target)
    
    % Params
    nState = 4;
    nGrid = 100;

    % Unpack decision variables
    tspan = z(1:2);
    x = reshape(z(3:nState*nGrid+2), nState, nGrid);
    u = z(nState*nGrid+3:end);
    dt = tspan(2)/(nGrid-1);
        
    % Evaluate the dynamics at each collocation point
    dx = model([], x, u);
    
    % Trapazoid rule:
    idxLow = 1:(nGrid-1);
    idxUpp = 2:nGrid;
    intStateTrap = 0.5*dt*(dx(:,idxLow) + dx(:,idxUpp));
    intStateCol = x(:,idxUpp)-x(:,idxLow);
    
    % Defect constraint:
    defect = intStateTrap - intStateCol;
    
    % Boundary Value Constraints
    BoundaryInit = x(:,1)-[0; pi; 0; 0];
    BoundaryFinal = x(:,end)-target;
    
    % Constraints
    C = [];
    Ceq = [BoundaryInit; reshape(defect,numel(defect),1); BoundaryFinal];

    % Outputs
    tSol = linspace(tspan(1), tspan(2), nGrid);
    zSol = x';

end


% Dynamics
function dx = model(~, x, u)

    % t - time
    % x - state vector [x; theta; x_dot; theta_dot]
    % u - input [F] (force applied to the cart)
    % m - mass of the cart (kg)
    % M - mass of the pole (kg)
    % L - length of the pole (m)
    % g - gravity constant (m/s^2)
    
    m = 10;  
    M = 1;  
    L = 1.0;  
    g = 9.81; 
    
    % State variables
    x_cart = x(1,:);
    theta = x(2,:);
    x_dot = x(3,:);
    theta_dot = x(4,:);
    
    % Equations of motion
    theta_acc = (g.*sin(theta) + cos(theta).*(-u-m*L.*theta_dot.^2.*sin(theta))/(M+m))./(L*(4/3 - m.*cos(theta).^2/(M+m)));
    x_acc = (u + m*L*(theta_dot.^2.*sin(theta)-theta_acc.*cos(theta)))/(M+m);

    % State derivative vector
    dx = [x_dot; theta_dot; x_acc; theta_acc];

end

% Objective function for optimization 
function cost = objective(u)

    % Initial energy of the cannon ball. 
    cost = norm(u);

end