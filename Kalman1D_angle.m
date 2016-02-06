function [ X ] = Kalman1D_angle( oQ, R, F, B, u, z, H, dt, t)
%1D kalman filtering of acc and loc as input
%   Detailed explanation goes here

disptime = 0;
 % Estimated state
X = [0;     % Alpha (angle)
     0];    % Bias
Xop = X;
%if(K)
%    precompK = true;
%else
%    precompK = false;

 %K = [0.05788; 0.01726];

 % Pre converge K 
 % Covariane martix
Q = oQ^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; 
P = eye(2).*Q;

if(disptime)
    tic
end

for k = 2:length(t)
    Pk = F*P*F' + Q;   % Covariance (size of the error) estimate
    S = H*Pk*H' + R;
    K = Pk*H'/S;
    P = (eye(2) - K*H)*Pk;
    Kvec(:,k) = K;
end

if(disptime)
    toc
    tic
end
 % Perform the Kalman filtering
for k = 2:length(t)
    % Predict
    Xk = F*X(:,k-1) + B*u(k); % State estimate
    
    % Update
    e = z(k) - H*Xk; % measurement residual (error)
    X(:,k) = Xk + K*e;
end

if(disptime)
    toc
    tic
end

 % Perform the Kalman filtering, optimized
for k = 2:length(t)
    % All in one step
    Xop(1,k) = Xop(1,k-1) - K(1)*(z(k) - Xop(1,k-1)) + (u(k)-X(2,k-1))*dt;
    Xop(2,k) = Xop(2,k-1) + K(2)*(z(k) - Xop(1,k-1));
end


if(disptime)
    toc
end

figure(10)
plot(Kvec');
plot(X'-Xop')
end

