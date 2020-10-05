function xrd = fcn(body_v,body_a, UUV_or,Ui,Uo)
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
     % xr - body rates state vector [q r]'
     % U - inner loop input control vector [delta_theta; delta_psi]
% OUTPUTS:
    % xrd - body rate dynamics derivative [q_dot r_dot]' 
% CONSTANTS:
% GLOBAL PARAMETERS:
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
% INPUT INITIALIZATION

% Body Fixed Velocity initialization
u = body_v(1);
v = body_v(2);
w = body_v(3);
p = body_v(4);
q = body_v(5);
r = body_v(6);
xr = [q r]';

% Body fixed acceleration initialization
ud = body_a(1);
vd = body_a(2);
wd = body_a(3);
pd = body_a(4);
qd = body_a(5);
rd = body_a(6);
        
% UUV orientation
phi = UUV_or(1)
theta = UUV_or(2);
psi = UUV_or(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN CODE:

% ATTITUDE DYNAMICS
% Ar, Br is the differential equation system of the body rate dynamics

Ar = [-(((Ix-Iz)*r*p+m*(zg*(ud-v*r+w*q)-xg*(wd-u*q+v*p)))+Mqd*qd+Mwd*wd+Mrp*r*p+Mq*u*q)/(Iy-Mqd)+(Mw*u*w+MwwQ*w*abs(w)+MqqQ*q*abs(q)-(xg*W-xb*B)*cos(theta)*cos(phi)-(zg*W-zb*B)*sin(theta))/(Iy-Mqd);...
    -(((Iy-Ix)*p*q+m*(xg*(vd-w*p+u*r)-yg*(ud-v*r+w*q)))+Nrd*rd+Nvd*vd+Npq*p*q+Nr*u*r)/(Iz-Nrd)+(Nv*u*v+NrrQ*r*abs(r)+NvvQ*v*abs(v)-(xg*W-xb*B)*cos(theta)*sin(phi)+(yg*W-yb*B)*sin(theta))/(Iz-Nrd)];

Br = [0 -Tp*xr/(Iy-Mqd) 0; 0 0 Tp*xr/(Iz-Nrd)];

% Differential Equation of Body rate dynamics
xrd = Ar + Br*Ui;

% POSITION DYNAMICS
% M matrix terms
M11 = -u*sin(theta)*sin(psi)+v*sin(phi)*cos(theta)*sin(psi)+w*cos(phi)*cos(theta)*sin(psi);
M12 = u*cos(theta)*cos(psi)+v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))+w*(sin(phi)*sin(psi)-cos(phi)*sin(theta)*cos(phi));
M21 = -u*cos(theta) - v*sin(theta)*sin(phi)-w*sin(theta)*cos(phi);
M22 = 0;

% N matrix terms
N11 = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))+w*(sin(theta)*cos(psi)*sin(psi)-sin(phi)*cos(psi));
N21 = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(psi)*cos(theta);

M = [M11 M12; M21 M22];
N = [N11; N21];

% dereivative of position vector of outer loop control
yz_d = M*Uo + N;