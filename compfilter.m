function [ X ] = compfilter( u, z, fact, dt)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

X = [z(1);0];
for k = 2:length(z)
    % Position
    Xk = X(1,k-1) + u(k)*dt;
    X(1,k) = Xk*(1-fact) + z(k)*fact;
    
    % Velocity
    X(2, k) = (X(1,k)-X(1,k-1))/dt;
end

