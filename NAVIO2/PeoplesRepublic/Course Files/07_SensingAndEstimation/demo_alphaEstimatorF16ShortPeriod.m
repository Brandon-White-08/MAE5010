%Simple Steady State Kalman Filter 
%Estimating alpha from normal accel nz and pitch rate q
%AP Design in class demo 24 Oct 2019
%Imraan A. Faruque i.faruque@okstate.edu
%
%Part 1: Estimate alpha from nz, q.
%Part 2: Add an actuator, how does this change the estimator?
%Part 3: Simulate the estimator in (2) with process +msnmt noise.
%
clear all; close all
%% F-16 short period approximation:
%x=[alpha;q], u=[delta_e]
A = [-1.01887, .90506; 0.82225, -1.07741];
B = [-0.00215; -0.17555];
G = [0.00203; -0.00164];
%Use imu measurements y = [nz; q]
C = [15.87875 1.48113; 0 1];

%First verify that this is a solvable problem
rank(obsv(A,C)) %should be n=2
cond(obsv(A,C)) %should not be large (close to unobservable)

%How big is the noise in w, v?  Quantify using variance
R = [1/20, 0; 0 1/60]; %measurement noise magnitude
Q = [1];               %process noise size
%Solve the A.R.E:
P = are(A',C'*inv(R)*C ,G*Q*G')
L = P*C'*inv(R)

%% Augment: Add an elevator actuator
%Use u->delta_e transfer function being 20.2/(s+20.2)
%x=[alpha; q; delta_e]
Aa = [A,B;
    zeros(1,2), -20.2];
Ba = [zeros(2,1);20.2];
Ga = [G;0];
%y = [nz;q]
Ca = [C,[0;0]];

%First verify that this is a solvable problem (observable)
rank(obsv(Aa,Ca)) %should be n=3, number of states
cond(obsv(Aa,Ca)) %should not be large (close to unobservable)

%Solve ARE
P = are(Aa',Ca'*inv(R)*Ca ,Ga*Q*Ga')
L = P*Ca'*inv(R)

%% Let's test this estimator with noisy inputs
%% First, simulate the noisy system

sys=ss(Aa,Ba,Ca,0); %form the system
%form the estimator
sensors=[1,2]; %use both sensors
known=[1];     %we know the only actuator input
est=estim(sys,L,sensors,known)%form the A-LC, etc

%Simulate system response to an input, include process noise
t=linspace(0,15,1e4); t=t';
u = sin(t)+.05*t;       %actuator input 
uact=u+sqrt(Q)*randn(1e4,1); %actual input w/ process noise
x0=[5*pi/180;4*pi/180;-3*pi/180];   %initial condition
[y,t,x]=lsim(sys,uact,t,x0);  %simulate dynamics
figure(1);plot(t,x); legend('\alpha','q','\delta_e')

%Corrupt the measurements with measurement noise
ym=y+[sqrt(R(1,1))*randn(size(y,1),1), sqrt(R(2,2))*randn(size(y,1),1)]; 
figure(2);
plot(t,y,'b-',t,ym,'c--',t,y,'b-'); 
legend('nz','q','nz_m','q_m'); xlabel('time,s');

%% Estimation: Run estimator
[estOut,t,estStates]=lsim(est,[u,ym],t); %estimate
%Note: estimate is not initialized to x0, only xhat=0, 
%so it starts with an estimate error that it must reject.
%It also doesn't know about uact.

%plotting
figure(3);subplot(2,1,1)
plot(t,x(:,1),'b-',t,estOut(:,3),'c:','linewidth',3);
legend('\alpha','\hat{\alpha}'); xlabel('time,s')';ylabel('rad')
subplot(2,1,2)
plot(t,x(:,2),'b-',t,estOut(:,4),'c:','linewidth',3);
legend('q','\hat{q}'); xlabel('time,s')';ylabel('rad/s')
