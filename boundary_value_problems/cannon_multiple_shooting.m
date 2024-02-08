% Compute the optimal trajectory of a canon ball to reach a given target 
% with minimal energy using Multiple Shooting. 

% Clear
clc
clear

% Target
target.x = 6;
target.y = 0;

%% Initial guess
v0 = 9; 
th0 = 45*(pi/180);

% Params
nState = 4;
nSegment = 10;
nSubStep = 5;
nGrid = nSegment * nSubStep;

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
[t, y] = ode45(userFun, tSpan, z0, options);

% Plot solution
%plot(y(:, 1),y(:, 2),'LineWidth',3); 
%xlabel('X  Position');
%ylabel('Y  Position');
%axis equal;
%xlim([0, 10])
%ylim([0, 5])

% Break the guess trajectory at segment bounds:
t_guess = linspace(0,t(end),nSegment+1); % Discretized time
t_guess(end) = []; % Remove last time stamp because this is a known state
z_guess = interp1(t, y, t_guess, 'spline')';

%% Multiple-shooting

% Store the initial guess for the problem
problem.x0 = [t(end), reshape(z_guess,1,nState*nSegment)];
problem.x0 = [10, ones(1,nState*nSegment)];
problem.lb = []; % Lower bound on decision variables
problem.ub = []; % Upper bound on decision variables

% Set up the linear constraints (there are none);
problem.Aineq = [];  problem.Aeq = [];
problem.bineq = [];  problem.beq = [];

% Set up the user-defined functions:
problem.objective = @(decVar)objective(decVar(4),decVar(5));  % Objective (cost) function
problem.nonlcon = @(decVar)nonLinCst(decVar,target);   % NonLinear constraints

% Set up the options for the solver:
problem.solver = 'fmincon';
problem.options = optimset('MaxFunEvals',1e4,'MaxIter',100,'Display','off');

% Use FMINCON to solve the constrained optimization problem:
[zSol, fVal, exitFlag] = fmincon(problem);

% Call the constraint function one final time to get the trajectory:
[~, ~, t, y] = nonLinCst(zSol,target);

% Plot trajectory in 2D
figure(1)
plot(y(:, 1),y(:, 2),'LineWidth',2); 
xlabel('X  Position');
ylabel('Y  Position');
axis equal;
xlim([0, 10])
ylim([t(1), t(end)])

% Plot trajectory in time
figure(2)
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

% Non-linear constraint function for multiple shooting 
function [C, Ceq, tSol, zSol] = nonLinCst(decVar,target)
    
    % Params
    nState = 4;
    nSegment = 10;
    nSubStep = 5;
    tEnd = decVar(1);
    
    % Run a simulation from the start of each segment, in parallel
    userFun = @(t,z)cannon_dynamics(t,z);
    tSpan = linspace(0, tEnd/nSegment, nSubStep+1);
    z0 = reshape(decVar(2:end), nState, nSegment);

    % Event options
    options = odeset('Events',@groundEvent,'Vectorized','on');
    
    % Solve
    tSol = [0];
    zSol = [];
    for i=1:nSegment
        [t, y] = ode45(userFun, tSpan, z0(:, i), options);
        plot(y(:,1), y(:,2)); hold on;
        tSol = [tSol; tSol(end) + t];
        zSol = [zSol; y];
    end
    tSol = tSol(2:end);

    % Plot
    axis equal;
    xlim([0, 10])
    ylim([0, 5])
    drawnow()
    
    % Boundary Value Constraints
    BoundaryInit = [zSol(1,1), zSol(1,2)];
    BoundaryFinal = [zSol(end,1) - target.x, zSol(end,2) - target.y];
    
    % Defect Constraints
    interDefects = zSol((1:nSegment-1)*(nSubStep+1), :) - zSol((1:nSegment-1)*(nSubStep+1)+1, :);
    interDefects = reshape(interDefects, 1, nState*(nSegment-1));
    
    % Define constraints
    C = [];  % No inequality constraints
    Ceq = [BoundaryInit, interDefects, BoundaryFinal];  % Boundary Condition

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

% Objective function for optimization 
function cost = objective(dx,dy)

    % Initial energy of the cannon ball. 
    cost = dx.*dx + dy.*dy;

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
