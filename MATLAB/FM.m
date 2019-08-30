%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019

function FM = FM(t)
%--------------------------------------------------------------------------
% This function is used to convert Euler Angles to Quaternions.
%
% Arguments:
%             t  (input)   time
%
%             EP (output)  Outputs Input function for derivative.m
%--------------------------------------------------------------------------
Fx = 2000*sin(t); % Set F in Body Frame
Fy = 0;
Fz = 0; % Set Fz other than gravity
L  = 0;
M  = 2000*1e-4;
N  = 0;

FM = [Fx,Fy,Fz,L,M,N];

