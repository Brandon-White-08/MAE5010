%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019

function EA = EP2Euler321(EP)
%--------------------------------------------------------------------------
% This function is used to convert Euler Angles to Quaternions.
%
%
% Arguments:
%             EP          (input)   Vector containing list of converted
%                                   Quanternion of form [q0,q1,q2,q2].
%
%             EulerAngles (Output)  Vector Containing angles corresponding
%                                   to [heading,pitch,roll] of vehical
%                                   orientation in degree.      
%--------------------------------------------------------------------------


% Sanity Check
if (length(EP) ~= 4)
    error('Please Check the length of Euler Quaternions in the Input');
end

% Assigning EP vector values to a different form
q0 = EP(1); q1 = EP(2); q2 = EP(3); q3 = EP(4);

% Quaternions to Euler 321 
phi = atan2d (2*(q0*q1+q2*q3),((q0^2)+(q3^2)-(q1^2)-(q2^2)));
theta = asind(2*(q0*q2 - q1*q3));
psi = atan2d (2*(q0*q3+q2*q1),((q0^2)+(q1^2)-(q3^2)-(q2^2)));

% Output
EA = [psi,theta,phi];

