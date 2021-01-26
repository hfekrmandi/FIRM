% PLOT_CREATOR.m
% Author: Ben Colvin
% Last Update: 1/26/2021
% Project: FIRM
% Purpose: Generate plots from the REMUS AUV. Specifically the 3D and 2D
% plots of the AUV's path
%--------------------------------------------------------------------------
% INSTRUCTIONS: 
% 1. Run Desired AUV model.
% 2. Check that X,Y,Z,tout updated in Matlab Workspace
% 3. Run Plot_Creator
%--------------------------------------------------------------------------
% VALUES:
% X - global X posistion vector over time
% Y - global Y posistion vector over time
% Z - global Z posistion vector over time
% tout - vector of time values for simulation
%--------------------------------------------------------------------------
% Close previous plots
close all

% Find length of the vectors
L = length(X);

% 3D Path Plot
figure(1)
% Plot 3D path
plot3(X,Y,Z)
grid

hold on
% Place circle on starting point
scatter3(X(1),Y(1),Z(1),'o')

% Place circle on ending point
scatter3(X(L),Y(L),Z(L),'o')

legend('Path','Start','End')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D Path')
hold off

% Create 2D path Plot
figure(2)
% Plot 2D path
plot(X,Y)
 hold on
 % Place circle at starting point
 scatter(X(1),Y(1),'o')
 % Place Circle at ending point
 
scatter(X(L),Y(L),'o')
legend('Path','Start','End')
xlabel('X')
ylabel('Y')
title('2D Path')
hold off