%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019
%--------------------------------------------------------------------------
% This file is used to rotate Wind frame to Body Frame
%--------------------------------------------------------------------------

function R_BW = R_BW(Angles)

  %Sanity Check
    if (length(Angles) ~= 2)
        error('Please Check the length of Angles in the Input');
    end
    alpha = Angles(1);
    beta  = Angles(2);
    
    R_BW = [[cosd(alpha)*cosd(beta),0,0];[sind(beta),0,0];[sind(alpha)*cosd(beta),0,0]];
    
end
        
    
   