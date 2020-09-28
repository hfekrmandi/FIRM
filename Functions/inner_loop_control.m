function [ui]=inner_loop_control(u,v,w,p,q,r,ud,vd,wd,pd,qd,rd, theta, phi, psi, xr, thetad, psid,theta_ref,theta_refd, psi_ref,psi_refd)
% Ben Colvin
% The purpose of this function is to calculate the inner loop control
% vector for the control of the UUV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
    % Body fixed terms:
        % u - surge velocity
        % v - sway velocity
        % w - heading velocity
        % p - roll velocity
        % q - pitch velocity
        % r - yaw velocity
        % ud - surge acceleration
        % vd - sway acceleration
        % wd - heading acceleration
        % pd - roll acceleration
        % qd - pitch acceleration
        % rd - yaw acceleration
    % World fixed terms:
        % theta - current orientation of UUV
        % psi - current orientation of UUV
        % xr - reference x position
        % thetad - current orientation velocity of UUV
        % psid - current orentation velocity of UUV
        % theta_ref - refernce orientation of UUV
        % theta_refd - reference orientation velocity of UUV
        % psi_ref - reference orientation of UUV
        % psi_refd - refernce orientation velocity of UUV
        
% OUTPUTS:
    % ui - control output of inner loop, to be used in the AUV dynamics

% Constants
a1 = 1; % control constant (not set) * must be +
a2 = 1; % control constant (not set) * must be +
k1 = 1; % control constant (not set)
k2 = 1; % control constant (not set)
%%% Other Constants are still being found, see Ar, Br for undefined
%%% constants
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main Code:
        
% Ar, Br is the differential equation system of the body rate dynamics
% xrd = Ar + Br*u, where u is the input to the DE
Ar = [-(((Ix-Iz)*r*p+m*(zg*(ud-v*r+w*q)-xg*(wd-u*q+v*p)))+Mqd*qd+Mwd*wd+Mrp*r*p+Mq*u*q)/(Iy-Mqd)+(Mw*u*w+Mww*w*abs(w)+Mqq*q*abs(q)-(xg*W-xb*B)*cos(theta)*cos(phi)-(zg*W-zb*B)*sin(theta))/(Iy-Mqd);...
    -(((Iy-Ix)*p*q+m*(xg*(vd-w*p+u*r)-yg*(ud-v*r+w*q)))+Nrd*rd+Nvd*vd+Npq*p*q+Nr*u*r)/(Iz-Nrd)+(Nv*u*v+Nrr*r*abs(r)+Nvv*v*abs(v)-(xg*W-xb*B)*cos(theta)*sin(phi)+(yg*W-yb*B)*sin(theta))/(Iz-Nrd)];

Br = [0 -Tp*xr/(Iy-Mqd) 0; 0 0 Tp*xr/(Iz-Nrd)];



% inner states error vector
ei = [theta - theta_ref; psi - psi_ref];

% inner state velocity error vector
eid = [thetad - theta_refd; psid - psi_refd];

% null control constants (not set)
k = [k1; k2];

% Positive Constant matrix (not set)
D = [a1 0; 0 a2];

% Transformation matrix for theta, psi
H = [cos(phi) -sin(phi); sin(phi)/cos(theta) cos(phi)/cos(theta)];

% Controls Coeffecient Row Vector funtion A_i(x_i,t)
Ai = 2*ei' * D * H *Br;

% Controls Load Scalar Function
Bi = [-2*ei't*D*(-xidd+Hd+H*(x^-1)); xi+H*Ar-2*eid'*D*eid; -2*c1*ei'*D*eid-c2*eid];

% MPGI of Ai
Ai_plus = Ai'/(Ai*Ai');

% null projection matrix
Pi = eye(2) - Ai_plus*Ai;

% null control vector
lamda = k*eid';

% Control output 2x1 matrix delta_theta, delta_psi 
ui = Ai_plus*Bi+Pi*lamda;