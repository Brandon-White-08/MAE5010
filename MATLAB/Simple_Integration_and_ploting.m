%Homework #1 (MAE 5010 Autopilot Design and Test)
%
%Name     :  Ujjval Patel
%Due Date :  08/29/2019

%--------------------------------------------------------------------------
% This flie Integrates the derivative function using ode45 (Runge-Kutta)
% method
%
% DO NOT FORGET TO KEEP ALL THE FILES IN ONE FOLDER BEFORE RUNNING
%
% state.mat, FM.mat, and MAV.mat has all the state variables needed as structure needed in
% derivative.m
%
%
%--------------------------------------------------------------------------

clear
clc
%% Loading the Variables and Setting the time frame
load('state.mat') %Initial Condition
load('MAV.mat')   %Paramaters for MAV
load('FM.mat')    %Force and Moment  

time_start = 0;
time_end   = 10;

ini = state;

%% Integrating the Derivative Function

[t,y]=ode45(@(t,y) derivatives(t,y,FM(t),MAV),[time_start time_end],ini);


%% Ploting
figure(1)

scatter3(y(:,1),y(:,2),y(:,3))
xlabel('Pn')
ylabel('Pe')
zlabel('Pd')