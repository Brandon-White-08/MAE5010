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
% DO NOT FORGET TO SET MAV PARAMETERS IN MAV.m and FORCE AND MOMENTS IN
% FM.m and the Initial condition for state at the bottom
%
% state.mat, FM.mat, and MAV.mat has all the state variables needed as 
% structure needed in
% derivative.m
%
%
%--------------------------------------------------------------------------

clear
clc

%% Set initial Condition
pn = 100;
pe = 200;
pd = -500;
u  = 0;
v  = 0;
w  = 0;
E = Euler3212EP([90,15,20]);
e0 = E(1);
e1 = E(2);
e2 = E(3);
e3 = E(4);
p  = 0;
q  = 0;
r  = 0;

ini = [pn,pe,pd, u,v,w, e0,e1,e2,e3, p,q,r];

%% Loading the Variables and Setting the time frame

time_start = 0;
time_end   = 1;

%% Integrating the Derivative Function

[t,y]=ode45(@(t,y) derivatives(t,y),[time_start time_end],ini);

%% Ploting
figure(1)
scatter3(y(:,1),y(:,2),y(:,3))
xlabel('Pn')
ylabel('Pe')
zlabel('Pd')

figure(2)
plot(t,y(:,4))
xlabel('Time(s)')
ylabel('u [m/s]')

figure(3)
plot(t,y(:,5))
xlabel('Time(s)')
ylabel('v [m/s]')

figure(4)
plot(t,y(:,6))
xlabel('Time(s)')
ylabel('w [m/s]')

figure(5)
plot(t,y(:,11))
xlabel('Time(s)')
ylabel('p [rad/s]')

figure(6)
plot(t,y(:,12))
xlabel('Time(s)')
ylabel('q [rad/s]')

figure(7)
plot(t,y(:,13))
xlabel('Time(s)')
ylabel('r [rad/s]')
