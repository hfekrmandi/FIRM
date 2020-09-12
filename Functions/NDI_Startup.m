% AUV/NDI Start Up code
% Defines parameters used in Simulink Model


m1 = 215; % mass (added+rigid) [kg]
m2 = 265; % mass (added+rigid) [kg]
m3 = 80; % inertia (added+rigid) [kg*m^2]

Xu = 70; % Surge Linear Drag [kg/s]
Xuu = 100; % Surge Quadratic Drag [kg/s]

Yv = 100; % Sway Linear Drag [kg/s]
Yvv = 200; % Sway Quadratic Drag [kg/m]

Nr = 50; % Yaw Linear Drag [kg*m^2/s]
Nrr = 100; % Yaw Quadratic Drag [kg*m^2]

Amp = 10;
Freq = 0.01;

Ur0 = 10^-3;    %Initial Surge Velocity Condition