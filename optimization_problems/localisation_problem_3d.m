% Satellites in the global positioning system (GPS) constellation continuously transmit a navigation message which includes 
% information about their orbital positions and the precise time. A GPS receiver that can get signals from at least four GPS 
% satellites can compute its location by solving the nonlinear algebraic equations defining the intersection of four spheres, 
% where the spheres' radii are determined by how long it took the messages to reach the receiver while traveling at the speed of light. 
% Technically, only three spheres are needed to resolve two points in space, one of which can typically be rejected as a candidate based 
% on its distance from the earth's surface. However, since GPS receivers' clocks are inaccurate compared to the atomic clocks on the satellites, 
% using a fourth satellite provides an extra equation that lets us additionally solve for the receiver s clock error.

% The equations are:
% (x - A1)^2 + (y-B1)^2 + (z-C1)^2 = (c*(t1 - Tr + e))^2
% (x - A2)^2 + (y-B2)^2 + (z-C2)^2 = (c*(t2 - Tr + e))^2
% (x - A3)^2 + (y-B3)^2 + (z-C3)^2 = (c*(t3 - Tr + e))^2
% (x - A4)^2 + (y-B4)^2 + (z-C4)^2 = (c*(t4 - Tr + e))^2

% Where numeric subscripts denote the satellite; A, B, and C; 
% t is the time when the satellite sent its navigation message;
% Tr is the receiver's time; e is the error in the receiver's clock; 
% and c is the speed of light, 299,792.458 km/s.

%    Ai [km]  Bi [km]  Ci [km]  ti-TR  
% 1	 15600	 7540	 20140	 0.07074
% 2	 18760	 2750	 18610	 0.07220
% 3	 17610	 14630	 13480	 0.07690
% 4  19170	 610	 18390	 0.07242

% Create data in kilometers [x, y, z, t]
data = [15600 7540 20140 0.07074; 
        18760 2750 18610 0.07220;
        17610 14630	13480 0.07690;
        19170 610 18390 0.07242];

% Speed of light
v = 300000; % km/s

% System
f = @(x) ((x(1) - data(:,1)).^2 + (x(2) - data(:,2)).^2 +  (x(3) - data(:,3)).^2) - ((v*(data(:,4) + x(4))).^2);

% Initial guess
x0 = [0; 0; 0; 0];

% Using fsolve
[sol,fval,exitflag,output,jacobian] = fsolve(f, x0)

%% Plot resultaat

% Get satellite data
[x,y,z] = sphere; % Standard sphere object
s1 = data(1,1:3); % [x, y, z] location of satelite S1
s2 = data(2,1:3); % [x, y, z] location of satelite S2
s3 = data(3,1:3); % [x, y, z] location of satelite S3
s4 = data(4,1:3); % [x, y, z] location of satelite S4

% Plot spheres
r_sat = 1000; % Radius satellite
r_aarde = 6371; % Radius earth
surf(x*r_sat + s1(1), y*r_sat + s1(2), z*r_sat + s1(3)); hold on; % Plot satelite S1
surf(x*r_sat + s2(1), y*r_sat + s2(2), z*r_sat + s2(3)); % Plot satelite S2
surf(x*r_sat + s3(1), y*r_sat + s3(2), z*r_sat + s3(3)); % Plot satelite S3
surf(x*r_sat + s4(1), y*r_sat + s4(2), z*r_sat + s4(3)); % Plot satelite S4
surf(x*r_aarde, y*r_aarde, z*r_aarde); % Plot earth

% Plot solution
plot3(sol(1), sol(2), sol(3), '*r');

% Plot satelite names
text(s1(1)+r_sat,s1(2),s1(3),"S1",'HorizontalAlignment','left','FontSize',10);
text(s2(1)+r_sat,s2(2),s2(3),"S2",'HorizontalAlignment','left','FontSize',10);
text(s3(1)+r_sat,s3(2),s3(3),"S3",'HorizontalAlignment','left','FontSize',10);
text(s4(1)+r_sat,s4(2),s4(3),"S4",'HorizontalAlignment','left','FontSize',10);

% Format
xlabel("X")
ylabel("Y")
zlabel("Z")
axis equal;