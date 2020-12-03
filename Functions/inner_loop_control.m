function [ui]=inner_loop_control(body_v,body_a, UUV_or, UUV_orV, or_ref, orV_ref, H,Hd, xr,xidd)
% Ben Colvin
% The purpose of this function is to calculate the inner loop control
% vector for the control of the UUV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
    % body_v - 6x1 vector of body fixed velocities:
        % u - surge velocity
        % v - sway velocity
        % w - heading velocity
        % p - roll velocity
        % q - pitch velocity
        % r - yaw velocity
    % body_a - 6x1 vector of body fixed accelerations
        % ud - surge acceleration
        % vd - sway acceleration
        % wd - heading acceleration
        % pd - roll acceleration
        % qd - pitch acceleration
        % rd - yaw acceleration
    % World fixed terms:
    % UUV_or - 2x1 vector orientation of UUV, theta, psi
        % theta - current orientation of UUV
        % psi - current orientation of UUV
    
    % UUV_orV - 2x1 vector of angular velocity thetad, psid
        % thetad - current orientation velocity of UUV
        % psid - current orentation velocity of UUV
    % or_ref -2x1 vector refernce orientation theta, psi
        % theta_ref - refernce orientation of UUV
        % psi_ref - reference orientation of UUV
    % orV_ref - 2x1 vector reference angular veloities thetad,psid
        % theta_refd - reference orientation velocity of UUV
        % psi_refd - refernce orientation velocity of UUV
    % H - 2x2 transformation matrix
    % Hd - 2x2 derivative of transformation matrix
    % xr - reference x position
        
% OUTPUTS:
    % ui - control output of inner loop, to be used in the AUV dynamics

% CONSTANTS
    a1 = 1; % control constant (not set) * must be +
    a2 = 1; % control constant (not set) * must be +
    k1 = 1; % control constant (not set)
    k2 = 1; % control constant (not set)
% GLOBAL PARAMETERS:
    global L
    global dia
    global m
    global Ix
    global Iy
    global Iz
    global Tp
    global Mwd
    global Mrp
    global Mq
    global Mqd
    global Mw
    global MwwQ
    global MqqQ
    global Nrd
    global Nvd
    global Npq
    global Nr
    global Nv
    global NrrQ
    global NvvQ
    global xg
    global xb
    global yg
    global yb
    global zg
    global zb
    global W
    global B
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT INITIALIZATION:

% Body Fixed Velocity initialization
u = body_v(1);
v = body_v(2);
w = body_v(3);
p = body_v(4);
q = body_v(5);
r = body_v(6);

% Body fixed acceleration initialization
ud = body_a(1);
vd = body_a(2);
wd = body_a(3);
pd = body_a(4);
qd = body_a(5);
rd = body_a(6);
        
% UUV orientation
theta = UUV_or(1);
psi = UUV_or(2);

% UUV angular velocities
thetad = UUV_orV(1);
psid = UUV_orV(2);

% UUV orientation reference
theta_ref = or_ref(1);
psi_ref = or_ref(2);

%UUV angular velocity reference
theta_refd = orV_ref(1);
psi_refd = orV_ref(2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Main Code:

% Ar, Br is the differential equation system of the body rate dynamics
% xrd = Ar + Br*u, where u is the input to the DE
Ar = [-(((Ix-Iz)*r*p+m*(zg*(ud-v*r+w*q)-xg*(wd-u*q+v*p)))+Mqd*qd+Mwd*wd+Mrp*r*p+Mq*u*q)/(Iy-Mqd)+(Mw*u*w+MwwQ*w*abs(w)+MqqQ*q*abs(q)-(xg*W-xb*B)*cos(theta)*cos(phi)-(zg*W-zb*B)*sin(theta))/(Iy-Mqd);...
    -(((Iy-Ix)*p*q+m*(xg*(vd-w*p+u*r)-yg*(ud-v*r+w*q)))+Nrd*rd+Nvd*vd+Npq*p*q+Nr*u*r)/(Iz-Nrd)+(Nv*u*v+NrrQ*r*abs(r)+NvvQ*v*abs(v)-(xg*W-xb*B)*cos(theta)*sin(phi)+(yg*W-yb*B)*sin(theta))/(Iz-Nrd)];

Br = [0 -Tp*xr/(Iy-Mqd) 0; 0 0 Tp*xr/(Iz-Nrd)];



% inner states error vector
ei = [theta - theta_ref; psi - psi_ref];

% inner state velocity error vector
eid = [thetad - theta_refd; psid - psi_refd];

% null control constants (not set)
k = [k1; k2];

% Positive Constant matrix (not set)
D = [a1 0; 0 a2];




% Controls Coeffecient Row Vector funtion A_i(x_i,t)
Ai = 2*ei' * D * H *Br;

% Controls Load Scalar Function
Bi = [-2*ei'*D*(-xidd+Hd+H*(x^-1)); xi+H*Ar-2*eid'*D*eid; -2*c1*ei'*D*eid-c2*eid];

% MPGI of Ai
Ai_plus = Ai'/(Ai*Ai');

% null projection matrix
Pi = eye(2) - Ai_plus*Ai;

% null control vector
lamda = k*eid';

% Control output 2x1 matrix delta_theta, delta_psi 
ui = Ai_plus*Bi+Pi*lamda;