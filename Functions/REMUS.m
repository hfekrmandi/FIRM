% REMUS.M Vehicle Simulator, returns the 
%   time derivative of the state vector
function [ACCELERATIONS,FORCES] = remus(x,ui)
% TERMS
% ---------------------------------------------------------------------
% STATE VECTOR:
% x = [u v w p q r xpos ypos zpos phi theta psi]'
% Body-referenced Coordinates
% u = Surge velocity [m/sec]
% v = Sway velocity [m/sec]
% w = Heave velocity [m/sec]
% p = Roll rate [rad/sec]
% q = Pitch rate [rad/sec]
% r = Yaw rate [rad/sec]
% Earth-fixed coordinates
% xpos = Position in x-direction [m]
% ypos = Position in y-direction [m]
% zpos = Position in z-direction [m]
% phi = Roll angle [rad]
% theta = Pitch angle [rad]
% psi = Yaw angle [rad]
% INPUT VECTOR
% ui = [delta_s delta_r]'
% Control Fin Angles
% delta-s = angle of stern planes [rad]
% delta-r = angle of rudder planes [rad]
% Initialize global variables
%7---------------------------------------------------------------------
%{
load vdata ;% W and B, CG and CB coords
load inv_mass_matrix ;% Minv matrix

load vehicle_coeffs ; %non-zero vehicle coefficients only
%}
% -------------------------------------------------------------------------
% Vehicle Parameters:
% -------------------------------------------------------------------------
m = 30.4791; % mass [kg]
W = +2.99e+002;% Measured Vehicle Weight [N]
B = +3.08e+002;  % Measured Vehicle Bouynacy  [N]
delta_max = 30; % Max deflection of control input

% -------------------------------------------------------------------------
% Table A.3: Center of Buoyancy wrt Origin at Vehicle Nose
% -------------------------------------------------------------------------
xb = -6.11e-001; %[m]
yb = +0.00e+000; %[m]
zb = +0.00e+000; %[m]

% -------------------------------------------------------------------------
% Table A.4: Center of Gravity wrt Origin at CB
% -------------------------------------------------------------------------
xg = +0.00e+000; % m
yg = +0.00e+000; % m
zg = +1.96e-002;% m

Ixx = 0.1770; % Inertia about x-axis [kg*m^2]
Iyy = 3.4500; % Inertia about y-axis [kg*m^2]
Izz = 3.4500; % Inertia about z-axis [kg*m^2]

% -------------------------------------------------------------------------
% Axial Drag Coefficient
% -------------------------------------------------------------------------
Xuu = -1.62e+000; %[kg/m]

% -------------------------------------------------------------------------
% Table C.2: Crossflow Drag Coefficients
% -------------------------------------------------------------------------
Yvv = -1.31e+003; % kg/m
Yrr = +6.32e-001; % kg -m/rad 
Zww = -1.31e+002; % kg/m 
Zqq = -6.32e-001; % kg m/rad2
Mwwd = +3.18e+000; % kg 
Mqq = -1.88e+002; % kg m2/rad
Nvvd = -3.18e+000; % kg
Nrr = -9.40e+001; % kg.m2/rad 2

% -------------------------------------------------------------------------
% Table C.3: Rolling Resistance Coefficient
% -------------------------------------------------------------------------
Kpp = -1.30e-001; % kg-m2/rad'

% -------------------------------------------------------------------------
% Table C.4: Body Lift and Moment Coefficients
% -------------------------------------------------------------------------
Yuv = -2.86e+001; % kg/m 
Zuw = -2.86e+001; % kg/m
Muwb = -4.47e+000; % kg
Nuvb = +4.47e+000; % kg
  

% -------------------------------------------------------------------------
% Table C.5: Added Mass Coefficients
% -------------------------------------------------------------------------
Xudot = -9.30e-001 ; % kg
Yvdot = -3.55e+001 ; % kg
Yrdot = +1.93e+000 ; % kg. m/rad
Zwdot = -3.55e+001 ; % kg
Zqdot = -1.93e+000 ; % kg. m/rad
Kpdot = -7.04e-002 ; % kg-m2 /rad
Mwdot = -1.93e+000 ; % kg. m  
Mqdot = -4.88e+000 ; % kg-m2 /rad
Nvdot = +1.93e+000 ; % kg-m
Nrdot = -4.88e+000 ; % kg-m2 /rad

% -------------------------------------------------------------------------
% Table C.6: Added Mass Force Cross-term Coefficients
% -------------------------------------------------------------------------
Xwq = -3.55e+001 ; % kg/rad
Xqq = -1.93e+000 ; % kg-m/rad
Xvr = +3.55e+001 ; % kg/rad
Xrr = -1.93e+000 ; % kg m/rad
Yura= -9.30e-001 ; % kg/rad
Ywp = +3.55e+001 ; % kg/rad
Ypq = +1.93e+000 ; % kg m/rad
Zuqa= +9.30e-001 ; % kg/rad
Zvp = -3.55e+001 ; % kg/rad
Zrp = +1.93e+000 ; % kg/rad

% -------------------------------------------------------------------------
% Table C.7: Added Mass K-Moment Cross-term Coefficients
% -------------------------------------------------------------------------
% ALL Zero 

% -------------------------------------------------------------------------
% Table C.8: Added Mass M-, N-Moment Cross-term Coefficients
% -------------------------------------------------------------------------
Muqa = +1.93e+000 ; % kg m/rad
Muwa = +3.46e+001  ; % kg
Mvp  = -1.93e+000 ; % kg-m/rad
Mrp  = +4.86e+000  ; % kg-m2/rad2
Nuva = -3.46e+001  ; % kg
Nura = +1.93e+000 ; % kg- m/rad
Nwp  = -1.93e+000  ; % kg- m/rad
Npq  = -4.86e+000  ; % kg- m2/rad2

% -------------------------------------------------------------------------
% Table C.9: Propeller Terms
% -------------------------------------------------------------------------
Xprop = +3.86e+000; % N
Kprop = -5.43e-001; % N-m

% -------------------------------------------------------------------------
% Table C.10: Table C.10: Control Fin Coefficients
% -------------------------------------------------------------------------

Yuudr = +9.64e+000 ; % kg/(m- rad)
Zuuds = -9.64e+000 ; % kg/(m- rad)
Muuds = -6.15e+000 ; % kg/rad
Nuudr = -6.15e+000 ; % kg/rad
Yuvf  = -9.64e+000 ; % kg/m
Zuwf  = -9.64e+000 ; % kg/m
Yurf  = +6.15e+000 ; % kg/rad
Zuqf  = -6.15e+000 ; % kg/rad
Muwf  = -6.15e+000 ; % kg
Nuvf  = +6.15e+000 ; % kg
Muqf  = -3.93e+000 ; % kg m/rad
Nurf  = -3.93e+000 ; % kg m/rad

% Missing Terms 
% ---------------------------------------------------------------------
% Table B.1: Non-Linear Force Coefficients
% ---------------------------------------------------------------------
Yur = Yura + Yurf; % Kg/rad Added Mass Cross Term and Fin Lift
Zuq = Zuqa + Zuqf; % Kg/rad Added Mass Cross Term and Fin Lift
Mww = Mwwd;
Nvv = Nvvd;
Muq = Muqa + Muqf;
Muw = Muwa + Muwf;
Nuv = Nuva + Nuvf; 
Nur = Nura + Nurf;

Minv = zeros(6,6);
Minv = [m-Xudot 0 0 0 m*zg -m*yg;...
    0 m-Yvdot 0 -m*zg 0 m*xg-Yrdot;...
    0 0 m-Zwdot m*yg -m*xg-Zqdot 0;...
    0 -m*zg m*yg Ixx-Kpdot 0 0;...
    m*zg 0 -m*xg-Mwdot 0 Iyy-Mqdot 0;...
    -m*yg m*xg-Nvdot 0 0 0 Izz-Nrdot]^-1;

% Output flags
show_forces = 10;
% Get and check state variables and control inputs
% Get state variables
u =x(1); v =x(2); w =x(3); p =x(4); q = x(5);
r = x(6); phi = x(10); theta = x(11); psi = x(12);

% Get control inputs
delta_s = ui(1); delta_r = ui(2);

% Check control inputs (useful later)
if delta_s > delta_max
delta_s = sign(delta_s)*delta_max;
end
if delta_r > delta_max   % This may cause issue - Ben C.
delta_r = sign(delta_r)*delta_max;
end

% Initialize elements of coordinate system transform matrix
%--------------------------------------------------------------------------
c1 = cos(phi);
c2 = cos(theta);
c3 = cos(psi);
s1 = sin(phi); 
s2 =sin(theta); 
s3 = sin(psi); 
t2 = tan(theta);

% Set total forces from equations of motion
%--------------------------------------------------------------------------
X = -(W-B)*sin(theta) + Xuu*u*abs(u) + (Xwq-m)*w*q + (Xqq +m*xg)*q^2 ...
+ (Xvr+m)*v*r + (Xrr + m*xg)*r^2 -m*yg*p*q - m*zg*p*r ...
+ Xprop;

Y = (W-B)*cos(theta)*sin(phi) + Yvv*v*abs(v) + Yrr*r*abs(r) +Yuv*u*v ...
+ (Ywp+m)*w*p + (Yur-m)*u*r - (m*zg)*q*r + (Ypq - m*xg)*p*q ...
+ Yuudr*u^2*delta_r;

Z = (W-B)*cos(theta)*cos(phi) + Zww*w*abs(w) + Zqq*q*abs(q)+Zuw*u*w ...
+ (Zuq+m)*u*q + (Zvp-m)*v*p + (m*zg)*p^2 + (m*zg)*q^2 ...
+ (Zrp - m*xg)*r*p + Zuuds*u^2*delta_s;

K = -(yg*W-yb*B)*cos(theta)*cos(phi) -(zg*W-zb*B)*cos(theta)*sin(phi) ...
+ Kpp*p*abs(p) - (Izz-Iyy)*q*r - (m*zg)*w*p + (m*zg)*u*r + Kprop;

M = -(zg*W-zb*B)*sin(theta) - (xg*W-xb*B)*cos(theta)*cos(phi) +Mww*w*abs(w) ...
+ Mqq*q*abs(q) + (Mrp - (Ixx-Izz))*r*p + (m*zg)*v*r - (m*zg)*w*q ...
+ (Muq - m*xg)*u*q + Muw*u*w + (Mvp + m*xg)*v*p ...
+ Muuds*u^2*delta_s;

N = -(xg*W-xb*B)*cos(theta)*sin(phi) - (yg*W-yb*B)*sin(theta) ...
+ Nvv*v*abs(v) + Nrr*r*abs(r) + Nuv*u*v ...
+ (Npq - (Iyy-Ixx))*p*q + (Nwp - m*xg)*w*p + (Nur + m*xg)*u*r ...
+ Nuudr*u^2*delta_r ;

FORCES = [X Y Z K M N]';
ACCELERATIONS =...
[Minv(1,1)*X+Minv(1,2)*Y+Minv(1,3)*Z+Minv(1,4)*K+Minv(1,5)*M+Minv(1,6)*N;...
Minv(2,1)*X+Minv(2,2)*Y+Minv(2,3)*Z+Minv(2,4)*K+Minv(2,5)*M+Minv(2,6)*N;...
Minv(3,1)*X+Minv(3,2)*Y+Minv(3,3)*Z+Minv(3,4)*K+Minv(3,5)*M+Minv(3,6)*N;...
Minv(4,1)*X+Minv(4,2)*Y+Minv(4,3)*Z+Minv(4,4)*K+Minv(4,5)*M+Minv(4,6)*N;...
Minv(5,1)*X+Minv(5,2)*Y+Minv(5,3)*Z+Minv(5,4)*K+Minv(5,5)*M+Minv(5,6)*N;...
Minv(6,1)*X+Minv(6,2)*Y+Minv(6,3)*Z+Minv(6,4)*K+Minv(6,5)*M+Minv(6,6)*N;...
c3*c2*u+(c3*s2*s1-s3*c1)*v+(s3*s1+c3*c1*s2)*w;...
s3*c2*u+(c1*c3+s1*s2*s3)*v+(c1*s2*s3-c3*s1)*w;...
-s2*u+c2*s1*v+c1*c2*w;...
p+s1*t2*q+c1*t2*r;...
c1*q-s1*r;...
s1/c2*q+c1/c2*r]; % There may be issues in this matrix as well