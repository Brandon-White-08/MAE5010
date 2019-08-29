function [t,p] = Integration()
    
    tspan = [tstart tend];
    Ini   = state;
    [t,p] = ode45(@derivatives