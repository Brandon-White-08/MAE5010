%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019

function EP = Euler3212EP(EulerAngles)
%--------------------------------------------------------------------------
% This function is used to convert Euler Angles to Quaternions.
%
% Arguments:
%             EulerAngles (input)   Vector Containing angles corresponding
%                                   to [heading,pitch,roll] of vehical
%                                   orientation in degree.
%
%             EP          (output)  Vector containing list of converted
%                                   Quanternion of form [q0,q1,q2,q2].
%--------------------------------------------------------------------------


% Sanity Check
if (length(EulerAngles) ~= 3)
    error('Please Check the length of Euler Angles in the Input');
end

% Convert and check if Euler Angles are in accaptable range.
phi = deg2rad(EulerAngles(3));
if (phi > pi || phi < -pi)
    error('Heading Out of range');
end

theta = deg2rad(EulerAngles(2));
if (theta > pi/2 || theta < -pi/2)
    error('Pitch Out of range');
end
psi = deg2rad(EulerAngles(1));
if (psi > pi || psi < -pi)
    error('Roll Out of range');
end

% Euler 321 to Quaternions
q0 = cos(psi/2)*cos(theta/2)*cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2);
q1 = cos(psi/2)*cos(theta/2)*sin(phi/2)-sin(psi/2)*sin(theta/2)*cos(phi/2);
q2 = cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*cos(theta/2)*sin(phi/2);
q3 = sin(psi/2)*cos(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2);

% Output
EP = [q0,q1,q2,q3];

