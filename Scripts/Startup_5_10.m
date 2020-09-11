% Ben Colvin
% 4/26/2020
% Under Supervision of Dr. Fekrmandi
% FIRM UUV Project
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code generates the state space model of the 3DOF UUV. It is
% currently linar. The Inputs to the system are 6 proppellers.

% Use the UUV_Model_4_29 to get the outputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UUV Model Parameters
% Sets the parameters of the UUV dynamic model
mxb = 175; % Rigid Body Mass in x DOF
mxa = mxb*0.5;  % Added body mass in x DOF, approximated
dx = 60;    % Linear Damping in x DOF
dxx = 90;   % Quadratic Damping in x DOF
gx = 0;     % Buoyancy in x DOF
mx = mxb+mxa;   % Total mass in x

mrb = 15;   % rigid body mass in psi
mra= mrb*0.5;   % added body mass in psi
dr = 20;    % linear damping in psi
drr = 15;   % quadratic damping in psi
gr =0;  % buoyancy in psi
mr = mrb+mra;   % total mass in psi

mzb = 175;  % rigid body mass in z
mza = mzb*0.5;  % added body mass in z
dz = 120; % Linear damping in z
dzz = 150;  % quadratic damping in z
gz = 0; % buoyancy in z
mz = mzb + mza; % total mass in z

kf = 0.0119; % torque coeffecient of proppelor/actuator
L = 0.4;    % Moment arm

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motor/Proppellor Parameters E-flite Power 25 motor
% https://www.electrocraft.com/products/bldc/RPX32/ -similar motor
% Sets the parameters of the motor
J = 0.00000111; % [kg*m^2] moment of inertia of the rotor
b = 0.1; % [N*m*s] motor viscous friction constant
Ke = 0.000207; % [V/rad/sec] electromotive force constant]
Kt = kf; % [N*m/Amp] motor torque constant
R = 0.1; % [ohm] electric resistance
H = 0.001; % [H] electric inductance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control
Kp_psi = 10;
Ki_psi = 1;
Kd_psi = 2;

Kp_x = 10;
Ki_x = 1;
Kd_x = 2;

Kp_z = 10;
Ki_z = 1;
Kd_z = 2;
% Main Code:

% Identifies parameters of state space 
states = {'x', 'psi', 'z', 'u', 'r', 'w'};
inputs = {'w1^2' 'w2^2' 'w3^2' 'w4^2' 'w5^2' 'w6^2'};
output = {'x' 'psi' 'z' 'u' 'r' 'w'};
% Sets up state matrix
A = [0 0 0 1 0 0;...
     0 0 0 0 1 0;...
     0 0 0 0 0 1;...
     0 0 0 -(dx+2*dxx)/mx 0 0;...
     0 0 0 0 -(dr+2*drr)/mr 0;...
     0 0 0 0 0 -(dz+2*dzz)/mz];
 B = [0 0 0 0 0 0;...
      0 0 0 0 0 0;...
      0 0 0 0 0 0;...
      kf/mx kf/mx 0 0 0 0;...
      kf*L/mr -kf*L/mr 0 0 0 0;...
      0 0 kf/mz kf/mz kf/mz kf/mz];
  C = [1 0 0 0 0 0;...
       0 1 0 0 0 0;...
       0 0 1 0 0 0;...
       0 0 0 1 0 0;...
       0 0 0 0 1 0;...
       0 0 0 0 0 1];
   D = zeros(6);
   
   % puts the system into state space form
   sys = ss(A,B,C,D,'StateName',states,'InputName',inputs,'OutputName',output)
   
   % gets controllability matrix
   c_matrix = ctrb(A,B);
   
   % if rank of the controllability matrix = 6 (full row rank), system is controllable
   rank_c = rank(c_matrix)
   
   % gets observability matrix
   o_matrix = obsv(A,C);
   
   % rank of the observability matrix, if = 6 (full column rank, system is
   % observable
   rank_o = rank(o_matrix)
   
   % Checks the eigen values of the system. If they are >0, system is
   % unstable
   eigen_values = eig(A)
   
   % Places the eigenvalues of the system using negative gain state
   % feedback
K = place(A,B,[-.89-.45*j,-0.89+.45*j,-8, -9, -10+7*j,-10-7*j])

% Controlled system
G = ss(A-B*K,B,C,D);

%figure(1)
%step(sys)

%figure(2)
%step(G)


