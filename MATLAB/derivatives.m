%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019

function xdot = derivatives (t,state, FM, MAV)
%--------------------------------------------------------------------------
% This function is used to convert Euler Angles to Quaternions.
%
%
% Arguments:
%             state       (input)   [pn,pe,pd, u,v,w, e0,e1,e2,e3, p,q,r]
%             FM          (input)   [Fx,Fy,Fz, L,M,N]    
%
%             xdot        (Output)  xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
%                                           e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];      
%--------------------------------------------------------------------------

%Assigning Variables

pn = state(1); pe = state(2); pd = state(3); u = state(4); v = state(5);
w = state(6); e0 = state(7); e1 = state(8); e2 = state(9); e3 = state(10);
p = state(11); q = state(12); r = state(13);

% m   = MAV.mass;
% Ix  = MAV.Ix;
% Iy  = MAV.Iy;
% Iz  = MAV.Iz;
% Ixz = MAV.Ixz;
% g   = MAV.g;
m = MAV(1); Ix = MAV(2); Iy = MAV(3); Iz = MAV(4); Ixz = MAV(5); g = MAV(6);

Fx = FM(1); Fy = FM(2); L  = FM(4); M  = FM(5); N  = FM(6); Fz = m*g;

% Convert Quaternion to Euler321
Angles = EP2Euler321([e0,e1,e2,e3]);
psi = Angles(1);
theta = Angles(2);
phi = Angles(3);

% position kinematics
P = [cosd(theta)*cosd(psi) sind(phi)*sind(theta)*cosd(psi)-cosd(phi)*...
    sind(psi) cosd(phi)*sind(theta)*cosd(psi)+sind(phi)*sind(psi); ...
    cosd(theta)*sind(psi) sind(phi)*sind(theta)*sind(psi)+cosd(phi)* ...
    cosd(psi) cosd(phi)*sind(theta)*sind(psi)-sind(phi)*cosd(psi); ...
    -sind(theta) sind(phi)*cosd(theta) cosd(phi)*cosd(theta)] * [u ; v; w];
pn_dot = P(1);
pe_dot = P(2);
pd_dot = P(3);

% position dynamics
U = [r*v-q*w; p*w-r*u; q*u-p*v] + ([Fx; Fy; Fz]./m);
u_dot = U(1);
v_dot = U(2);
w_dot = U(3);

% rotational kinematics
E = (1/2).*[0 -p -q -r;p 0 r -q;q -r 0 p;r q -p 0]*[e0;e1;e2;e3];
e0_dot = E(1);
e1_dot = E(2);
e2_dot = E(3);
e3_dot = E(4);

% rotational dynamics
I  = Ix*Iz - Ixz^2;
I1 = Ixz*(Ix-Iy+Iz)/I;
I2 = (Iz*(Iz-Iy)+Ixz^2)/I;
I3 = Iz/I;
I4 = Ixz/I;
I5 = (Iz-Ix)/Iy;
I6 = Ixz/Iy;
I7 = ((Ix-Iy)*Ix+Ixz^2)/I;
I8 = Ix/I;

p_dot = I1*p*q - I2*q*r + I3*L+I4*N;
q_dot = I5*p*r - I6*(p^2-r^2) + M/Iy;
r_dot = I7*p*q -I1*q*r + I4*L + I8*N ;

% collect all the derivaties of the states
xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;e0_dot; e1_dot;...
    e2_dot; e3_dot; p_dot; q_dot; r_dot];
end

