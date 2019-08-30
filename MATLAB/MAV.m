%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019
%--------------------------------------------------------------------------
% This file is used as a Input of MAV Parameters for derviative.m
%
%--------------------------------------------------------------------------


function mav = MAV()
    
    m  = 2750;       
    Ix = 1048;
    Iy = 3000;
    Iz = 3530;
    Ixz = 0;
    g = 9.81;
    
    mav = [m Ix Iy Iz Ixz g];
end