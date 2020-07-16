clear 
close all 
clc 

% Mass matrix
syms mass %= 175; [kg]
syms Ixx  %= 14;  [kg m^2]
syms Iyy  %= 13;    [kg m^2]
syms Izz  %= 16;   %[kg m^2]

% Added mass matrix
syms Xdu  %= 120;  %[Ns/m]
syms Ydv  %= 90;   %[Ns/m]
syms Zdw  %= 150;  %[Ns/m]
syms Kdp  %= 0;
syms Mdq  %= 0;
syms Ndr  %= 18;   %[Ns/m]

% Damping matrix
syms Xu   %= 0;
syms Yv   %= 0;
syms Zw   %= 0;
syms Kp   %= 0;
syms Mq   %= 0;
syms Nr   %= 0;

% Quadratic damping matrix
syms Xuu  %= 90;   %[Ns^2/m^2]
syms Yvv  %= 90;   %[Ns^2/m^2]
syms Zww  %= 120;  %[Ns^2/m^2]
syms Kpp  %= 0;
syms Mqq  %= 0;
syms Nrr  %= 15;   %[Ns^2/m^2]


syms xg   %= 0;
syms xb   %= 0;
syms yg   %= 0;
syms yb   %= 0;
syms zg   %= 0;
syms zb   %= 100;
syms theta %= 0;
syms phi  %= 0;

% UUV specific 
syms W;% = mass*9.81;  % [N] - weight
syms dens;% = 997; %[kg/m^3] - density
syms V;% = 151; %[L] - volume [V = V*0.001;    %Convert to m^3]
syms B ;%= dens*9.81*V;    %Buoyancy
syms Cd ;%= 0.5;   %drag coeffeceint estimate
syms Af ;%= 1; %cross Section?

% State variables 
%  first derivatives of displacement and angles 
syms u v w p q r 

% State derivatives
% second  derivatives of displacement and angles 
syms u_dot v_dot w_dot p_dot q_dot r_dot 

% torque and force matrices 
syms l1 l2 l3 l4 l5 l6 % 1 2 l = l = 400mm, 3 5 l = 50mm, l = 400mm and 6 l = 600mm
syms T1 T2 T3 T4 T5 T6 

% AUV is not symmetric about the x-y plane it can be assumed to 
% be symmetric because the vehicle operates at relative low speeds
M_rb = [mass 0 0 0 0 0;...
            0 mass 0 0 0 0;...
            0 0 mass 0 0 0;...
            0 0 0 Ixx 0 0;...
            0 0 0 0 Iyy 0;...
            0 0 0 0 0 Izz];

% parameters of the added mass matrix are constant 
% when the vehicle is fully submerged
M_a = [Xdu 0 0 0 0 0;...
            0 Ydv 0 0 0 0;...
            0 0 Zdw 0 0 0;...
            0 0 0 Kdp 0 0;...
            0 0 0 0 Mdq 0;...
            0 0 0 0 0 Ndr];
% Total mass        
M = M_rb + M_a;

% Hydrodynamic damping matrix
D_v = [Xu 0 0 0 0 0;...
                  0 Yv 0 0 0 0;...
                  0 0 Zw 0 0 0;...
                  0 0 0 Kp 0 0;...
                  0 0 0 0 Mq 0;...
                  0 0 0 0 0 Nr];
% Quadratic damping matrix              
D_q = [Xuu 0 0 0 0 0;...
                  0 Yvv 0 0 0 0;...
                  0 0 Zww 0 0 0;...
                  0 0 0 Kpp 0 0;...
                  0 0 0 0 Mqq 0;...
                  0 0 0 0 0 Nrr];

% Gravitational and buoyancy matrix 
g = [(W-B)*sin(theta);...
        -(W-B)*cos(theta)*sin(phi);...
        -(W-B)*cos(theta)*sin(phi);...
        -(yg*W-yb*B)*cos(theta)*cos(phi)+(zg*W-zb*B)*cos(theta)*cos(phi);...
        (zg*W-zb*B)*sin(theta)-(xg*W-xb*B)*cos(theta)*cos(phi);...
        -(xg*W-xb*B)*cos(theta)*sin(phi)-(yg*W-yb*B)*sin(theta)]

% Force and Torque mapping matrix 
L = [1 1 0 0 0 0;...
                  0 0 0 0 0 0;...
                  0 0 1 1 1 1;...
                  0 0 -l1 l2 -l1 l2;...
                  l3 l3 -l6 l6 -l5 l5;...
                  -l1 l2 0 0 0 0]
% Trust Vector 
U = [T1; T2; T3; T4; T5; T6]

% Force and Torque vector 
Tau = L * U

% State vectors and derivatives
X = [u; v; w; p; q; r]
X_dot = [u_dot; v_dot; w_dot; p_dot; q_dot; r_dot]

% State representation from governing differential equation
% u v w p q r
X_dot = inv(M)*[ Tau - D_v*X - D_q*X.*abs(X) -g ]


