function [ X ] = Kalman1D( oQ, R, F, B, u, z, H, dt, t)
%1D kalman filtering of acc and loc as input
%   Detailed explanation goes here


 % Estimated state
X = [0;     % Location
     0];    % Speed

%if(K)
%    precompK = true;
%else
%    precompK = false;

 %K = [0.05788; 0.01726];
 
 % Covariane martix
Q = oQ^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; 
P = eye(2).*Q;
 % Perform the Kalman filtering
for k = 2:length(t)
    % Predict
    Xk = F*X(:,k-1) + B*u(k); % State estimate
    Pk = F*P*F' + Q;   % Covariance (size of the error) estimate
    
    % Update
    e = z(k) - H*Xk; % measurement residual (error)
    
    S = H*Pk*H' + R;
    K = Pk*H'/S;
    
    X(:,k) = Xk + K*e;
    P = (eye(2) - K*H)*Pk;
    
    Kvec(:,k) = K;
end

figure(10)
plot(Kvec');
end

