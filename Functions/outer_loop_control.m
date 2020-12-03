function Uo =outer_loop_control(body_v,eo, eod,ydd,zdd)
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
    % eo - 2x1 state error vector [y_e; z_e]
    % eod - 2x1 velocity error vector [yd_e; zd_e]
    % ydd - acceleration in y direction of global system
    % zdd - acceleration in z direction of global system
    
% OUTPUTS:
    % Uo - control output of outer loop, to be used in the AUV dynamics

% CONSTANTS
    a1 = 1; % control constant (not set) * must be +
    a2 = 1; % control constant (not set) * must be +
    a3 = 1; % control constant (not set)
    a4 = 1; % control constant (not set)
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

% Outer Position Acceleration Vector
xo_dd = [ydd; zdd];

% Control Constants
D12 = [a1 0; 0 a2];
D34 = [a3 0; 0 a4];
k_o = [k1; k2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN CODE:

% M matrix terms
M11 = -u*sin(theta)*sin(psi)+v*sin(phi)*cos(theta)*sin(psi)+w*cos(phi)*cos(theta)*sin(psi);
M12 = u*cos(theta)*cos(psi)+v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))+w*(sin(phi)*sin(psi)-cos(phi)*sin(theta)*cos(phi));
M21 = -u*cos(theta) - v*sin(theta)*sin(phi)-w*sin(theta)*cos(phi);
M22 = 0;

N11 = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))+w*(sin(theta)*cos(psi)*sin(psi)-sin(phi)*cos(psi));
N21 = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(psi)*cos(theta);

M = [M11 M12; M21 M22];
N = [N11; N21];



% Dynamics
Ao = 2*eo'*D34*M;
Bo = 2*eo'*D12*(xo_dd-N)+2*eod'*D12*eod+2*c1*eo'*D12*eod+c2*eo'*D12*eo;

% Adjunct Matrix A*
Ao_star = adjoint(Ao);

% Moore-Penrose Generealized Inverse
Ao_plus = (Ao_star*Ao)^-1 * Ao_star;

% Null Projection Matrix for outer loop
Po = eye(2) - Ao_plus*Ao;

% Null Control Vector
lamda_o = k_o*eod';

% Outer Loop Control Input [triangle_psi; triagnle _theta]
Uo = Ao_plus*Bo+Po*lamda_o;

