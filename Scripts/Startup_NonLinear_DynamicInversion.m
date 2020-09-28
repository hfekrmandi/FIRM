% Ben Colvin
% FIRM Research Project
% Nonlinear Dynamic Inversion Control of a AUV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code will act as the startup code to initialoze parameters in the
% workspace
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The AUV is modeled after the Monterey Bay Aquarium Research Institute (MBARI)

%
L = 5.554   % length [meters]
D = 0.533   % Diameter [meters]
m = 1093.1  % mass [kg]
Ix = 36.677 % moment of inertia about Xb [kg*m^2]
Iy = 2154.3 % Moment of inertia baout Yb [kg*m^2]
Iz = 2154.3 % Moment of inertia about Zb [kg*m^2]
Tp = 52     % Thrust [N]
% Max deflection of delta_theta and delta_psi = 15 deg