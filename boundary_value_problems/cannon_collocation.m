% Compute the optimal trajectory of a canon ball to reach a given target 
% with minimal energy using Direct Collocation. 

%% Init

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
nGrid = 50+1;

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

% Break the guess trajectory at segment bounds:
t_guess = linspace(0,t(end),nGrid); % Discretized time
z_guess = interp1(t, y, t_guess, 'spline')';

% Plot solution
%plot(y(:, 1),y(:, 2)); hold on;
%plot(z_guess(:, 1),z_guess(:, 2),'*'); 
%xlabel('X  Position');
%ylabel('Y  Position');
%axis equal;
%xlim([0, 10])
%ylim([0, 5])

%% Setup solver

% Store the initial guess for the problem
problem.x0 = [t(end), reshape(z_guess,1,nState*nGrid)];
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

%% Solve 

% Use FMINCON to solve the constrained optimization problem:
[zSol, fVal, exitFlag] = fmincon(problem);

%% Get solution

% Call the constraint function one final time to get the trajectory:
[~, ~, t, y] = nonLinCst(zSol,target);
length(t)
length(y)

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

% Non-linear constraint function for collocation
function [C, Ceq,tSol,zSol] = nonLinCst(decVar,target)

    % Params
    nState = 4;
    nGrid = 50+1;
    tEnd = decVar(1);
    zSol = reshape(decVar(2:end),nState,nGrid);
    
    % Parallel quadrature integration between each pair of collocation points
    dt = tEnd/(nGrid-1);
    d = getDefect(dt,zSol);

    % Plot
    %plot(zSol(1,:), zSol(2,:), "*")
    %axis equal;
    %xlim([0, 10])
    %ylim([0, 5])
    %drawnow()
    
    % Boundary Value Constraints
    BoundaryInit = [zSol(1,1); zSol(2,1)];
    BoundaryFinal = [zSol(1,end) - target.x, zSol(2,end) - target.y]';
    
    % Pack up the constraints:
    C = [];  %No inequality constraints
    Ceq = [BoundaryInit; reshape(d,nState*(nGrid-1),1); BoundaryFinal];

    % Time vector
    tSol = linspace(0,tEnd,nGrid);
    zSol = zSol';

end

% Compute the Defects - Direct Collocation
function d = getDefect(dt,z)

    %
    % Compute defects via hermite simpson quadrature:
    %
    % f = dynamics(t,z)
    %
    % d(k) = z(k+1) - z(k) - (dt/6)*(f(k) + 4*fBar(k+1) + f(k+1))
    %  
    % fBar(k+1) = dynamics(t(k) + dt/2, zBar(k+1))
    %
    % zBar(k+1) = (1/2)*(z(k)+z(k+1)) + (dt/8)*(f(k) - f(k+1))
    %
    % Reference:
    % "A Survey of numerical methods for optimal control"
    %  - Anil Rao, 2009, Advances in Astronautical Sciences
    %
    
    %%% Some useful quantities:
    zLow = z(:,1:(end-1));
    zUpp = z(:,2:end);
    
    %%% Compute the dynamics at each grid point:
    f = cannon_dynamics([],z);  % no time dependence
    fLow = f(:,1:(end-1));
    fUpp = f(:,2:end);
    
    %%% Compute the cannon dynamics at intermediate point:
    zBar = 0.5*(zLow + zUpp) + (dt/8).*(fLow-fUpp);
    fBar = cannon_dynamics([],zBar);
    
    %%% Compute the defects:
    d = zUpp - zLow - (dt/6).*(fLow + 4*fBar + fUpp);

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
