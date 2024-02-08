% The movement of planets (and the sun) is governed by Newton's laws of gravitation. First, the
% acceleration a is equal to the force F divided by the mass m (i.e. F = ma). Second, the gravitational
% force which two masses m1 and m2 exert on each other is given by
% Fz = m1m2G/r^2 ;
% where r is the distance between the masses and G = 6:672*10^-11 Nm2kg^-2 is the gravitational
% constant. The direction of the force is of course along the line connecting the masses.
% The input parameters are a vector m with the masses of the planets (say of length p), a matrix
% of initial positions (n x p), a matrix of initial velocities (n x p), and the length of time T over
% which the movements should be computed. The function makes a picture of the orbits and returns
% as output a vector of points in time and a matrix of corresponding positions and velocities of the
% planets.

% Clear
clc
clear
close all

% Params
tspan = [0 10000];

% Planet 1 (Sun)
m_1 = 1;
y_1_pos = [0, 0, 0]';
y_1_vel = [0, 0, 0]';

% Planet 2 (Jupiter)
m_2 = 0.000954786104043;
y_2_pos = [-3.5023653, -3.8169847, -1.5507963]';
y_2_vel = [0.00565429, -0.00412490, -0.00190589]';

% Planet 3 (Saturn)
m_3 = 0.000285583733151;
y_3_pos = [9.0755314, -3.0458353, -1.6483708]';
y_3_vel = [0.00168318, 0.00483525, 0.00192462]';

% Planet 4 (Uranus)
m_4 = 0.0000437273164546;
y_4_pos = [8.3101420, -16.2901086, -7.2521278]';
y_4_vel = [0.003541788, 0.00137102, 0.00055029]';

% Planet 5 (Neptune)
m_5 = 0.0000517759138449;
y_5_pos = [11.4707666, -25.7294829, -10.8169456]';
y_5_vel = [0.00288930, 0.00114527, 0.00039677]';

% Planet 6 (Pluto)
m_6 = 1/(1.3*10^8);
y_6_pos = [-15.5387357, -25.2225594, -3.1902382]';
y_6_vel = [0.00276725, -0.00170702, -0.00136504]';

% Planet 7 (Earth)
m_7 = 0;
y_7_pos = [0, 0, 0]';
y_7_vel = [0, 0, 0]';

% Planet 8 (Moon)
m_8 = 0;
y_8_pos = [0, 0, 0]';
y_8_vel = [0, 0, 0]';

% Initial state vector
m = [m_1 m_2 m_3 m_4 m_5 m_6];
init_pos = [y_1_pos y_2_pos y_3_pos y_4_pos y_5_pos y_6_pos];
init_vel = [y_1_vel y_2_vel y_3_vel y_4_vel y_5_vel y_6_vel];

% Params
p = size(m, 2);
G = 2.95912208286*10e-4; %m^3kg^-1s-2 6.67430*10e-11;

% State vector
y_0 = [init_pos(1:3,:); init_vel(1:3,:)];
y_0 = reshape(y_0, 6*p, 1);

% History
global pi_hist;
pi_hist = [];

% Ode solver
[t, y] = ode45(@(t,y) odefun(t, y, G, m), tspan, y_0);
    
% Extract x-, y-, and z- positions in time for all planets
y_x = y(:, 1:2*3:size(y_0, 1));
y_y = y(:, 2:2*3:size(y_0, 1));
y_z = y(:, 3:2*3:size(y_0, 1));

% Plot result
figure(1)
for k = 1:numel(t)
    plot3(y_x(k,1),y_y(k,1),y_z(k,1), 'y.', 'MarkerSize', 30); hold on;
    plot3(y_x(k,2),y_y(k,2),y_z(k,2), 'r.');
    plot3(y_x(k,3),y_y(k,3),y_z(k,3), 'g.');
    plot3(y_x(k,4),y_y(k,4),y_z(k,4), 'b.');
    plot3(y_x(k,5),y_y(k,5),y_z(k,5), 'c.');
    plot3(y_x(k,6),y_y(k,6),y_z(k,6), 'm.');
    %drawnow;
end

function y_dot = odefun(~, y, G, m)
    
    % State vector length
    len_y = length(y);
    
    % Reshape y-vector
    y = reshape(y, 6, length(m));
    pi_ = y(1:3, :);
    qi = y(4:6, :);
   
    % Init y_dot
    pi_dot = qi;
    qi_dot = zeros(size(qi));
    
    % Loop over all planets
    for i=1:size(pi_, 2)
        
        % Fill in accelerations for each planet
        force = zeros(3, 1);
        for j=1:size(pi_, 2)
            if i~=j
                
                % Position vector from planet i to planet j
                r = pi_(:, j) - pi_(:, i);
                
                % Distance between planets i and j
                distance = norm(r);
                
                % Unit position vector from planet i to planet j
                unit_r = r./distance;

                % Force on planet i induced by planet j
                force_ij = (G * m(i) * m(j) / distance^2) .* unit_r;
                
                % Add to total force
                force = force + force_ij;
                
            end
        end
        
        % Fill in accelerations
        qi_dot(:,i) = force ./ m(i);
        
    end
    
    % Create y_dot-vector
    y_dot = [pi_dot; qi_dot];
    y_dot = reshape(y_dot, len_y, 1);

    % Save in history
    global pi_hist;
    pi_hist = [pi_hist; pi_*10000];

    % Plot
    [x,y,z] = sphere;
    s1 = pi_(:,1)*10000;
    s2 = pi_(:,2)*10000;
    s3 = pi_(:,3)*10000;
    s4 = pi_(:,4)*10000;
    s5 = pi_(:,5)*10000;
    s6 = pi_(:,6)*10000;

    % Radius planets
    r_1 = 69.6*100;
    r_2 = 69*100;
    r_3 = 58*100;
    r_4 = 25*100;
    r_5 = 24*100;
    r_6 = 1*100;
    
    % Plot as surface.
    %surf(x*r_1 + s1(1),y*r_1 + s1(2),z*r_1 + s1(3)); hold on;
    %surf(x*r_2 + s2(1),y*r_2 + s2(2),z*r_2 + s2(3)) 
    %surf(x*r_3 + s3(1),y*r_3 + s3(2),z*r_3 + s3(3)) 
    %surf(x*r_4 + s4(1),y*r_4 + s4(2),z*r_4 + s4(3)) 
    %surf(x*r_5 + s5(1),y*r_5 + s5(2),z*r_5 + s5(3)) 
    %surf(x*r_6 + s6(1),y*r_6 + s6(2),z*r_6 + s6(3)) 
    text(s1(1),s1(2),s1(3),["Sun"],'HorizontalAlignment','left','FontSize',10); hold on;
    text(s2(1),s2(2),s2(3),["Jupyter"],'HorizontalAlignment','left','FontSize',10);
    text(s3(1),s3(2),s3(3),["Saturn"],'HorizontalAlignment','left','FontSize',10);
    text(s4(1),s4(2),s4(3),["Uranus"],'HorizontalAlignment','left','FontSize',10);
    text(s5(1),s5(2),s5(3),["Neptune"],'HorizontalAlignment','left','FontSize',10);
    text(s6(1),s6(2),s6(3),["Pluto"],'HorizontalAlignment','left','FontSize',10);

    % Plot path 3D
    plot3(pi_hist(1:3:end,1), pi_hist(2:3:end,1), pi_hist(3:3:end,1), '.k')
    plot3(pi_hist(1:3:end,2), pi_hist(2:3:end,2), pi_hist(3:3:end,2), '.k')
    plot3(pi_hist(1:3:end,3), pi_hist(2:3:end,3), pi_hist(3:3:end,3), '.r')
    plot3(pi_hist(1:3:end,4), pi_hist(2:3:end,4), pi_hist(3:3:end,4), '.g')
    plot3(pi_hist(1:3:end,5), pi_hist(2:3:end,5), pi_hist(3:3:end,5), '.b')
    plot3(pi_hist(1:3:end,6), pi_hist(2:3:end,6), pi_hist(3:3:end,6), '.k')

    % Plot
    drawnow;
    pause(0.1);
    clf;
  
end

