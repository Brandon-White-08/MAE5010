%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019
%--------------------------------------------------------------------------
% This file is used to rotate Fixed frame to Body Frame
%--------------------------------------------------------------------------

function R_BF = R_BF(Angles)

    psi = Angles(3);
    theta = Angles(2);
    phi = Angles (1);
    
    R_BF = [[cosd(psi)*cosd(theta), sind(psi)*cosd(theta), -sind(theta)]; ...
        [cosd(psi)*sind(theta)*sind(phi)-sind(psi)*cosd(phi), ...
        cosd(psi)*cosd(phi)+sind(theta)*sind(psi)*sind(phi),cosd(theta)*sind(phi)]; ...
        [cosd(psi)*sind(theta)*sind(phi)+sind(phi)*sind(psi),sind(theta)*sind(psi)* ...
        cosd(phi) - sind(phi)*cosd(psi),cosd(theta)*cosd(phi)]];
end
        
    
   