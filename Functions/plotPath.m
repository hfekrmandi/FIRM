function plotPath(x,y,z,waypoints)
% plotPath.m    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the path of the UUV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate figure
figure;
% Title figure
title('Path')
% Create 3d plot
plot3(x,y,z);
hold on;
% Add waypoints as circles to plot
scatter3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'MarkerEdgeColor',...
    [0.8500,0.3250,0.0980]);
% Label axis
xlabel('$x$ (m)','Interpreter','Latex');
ylabel('$y$ (m)','Interpreter','Latex');
zlabel('$z$ (m)','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');

end
