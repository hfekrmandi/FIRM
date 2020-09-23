% Author: Ben Colvin
% Project: FIRM UUV
% Advisor: Dr. Hadi Fekrmandi
% Date: 6/28/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is the startup code used to define workspace variables before 
% running the UUV Simulink model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Current Status of Model
    % PID paramters have been tuned for a step input. Overshoots are less
    % than 20% using the PID paramters below.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Future of the Model
    % The next step is developing a path planning scheme to track and guide
    % the AUV. 
    % Fault Identification and Tolerance will then be implemented
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mx = 160;   % Mass of UUV
dx = 60;    % Linear Damping
dxx = 90;   % Quadratic Damping term
g = 0;      % buoyancy
kpx = 35;   % proportional gain
kix=15;      % integral gain
kdx = 22;   % derivative gain
mr = 15;    % Izz of UUV
dr = 20;    % Linear Damping
drr=15;     % Quadratic Damping
kpr=25;     % Proportional Gain
kdr=20;     % Derivative Gain
kir = 7;    % Integral Gain
kf = 0.0019;    % Motor Force Coeffecient
L = 0.4;        % distance from propeller to center of gravity
mz = 160;       % mass of UUV
dz = 60;        % Linear Damping
dzz = 90;       % Quadratic Damping
kpz = 25;       % Proportional gain
kiz = 5;        % Integral Gain
kdz = 20;       % Derivative Gain

kps = 25;
kds=10;
kis=8;

waypoints = [0,0,0;
             -1,-1,-1];
        
r = 0.15;

Ud = 0.5;



plotPath(xout,yout,zout,waypoints);
figure();
plot(xout,yout)
hold on
scatter(waypoints(:,1),waypoints(:,2),'MarkerEdgeColor',...
    [0.8500,0.3250,0.0980]);
xlabel('x, (m)')
ylabel('y, (m)')
grid
title('Horizantal Plane Navigation')

hold off
