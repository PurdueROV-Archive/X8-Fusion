function [ X ] = Kalman1D_IMU( u, z, dt, K )
% 1D rotational Kalman filtering of IMU data
%   u = Gyro
%   z = estimated rotation from Acc and Mag data
%   K = Pre computed Kalman gain

 % Estimated state
X = [z(1);     % Alpha (angle)
     0];    % Bias

 % Intinalize
F = [1, -mean(dt);     % State transistion
     0, 1];

B = [mean(dt) ;
     0 ];
 
H = [1, 0]; % Observation model

tic
for k = 2:length(dt)
    % Predict
    Xk = F*X(:,k-1) + B*u(k); % State estimate
    
    % Update
    e = z(k) - H*Xk; % measurement residual (error)
    X(:,k) = Xk + K*e;
end
toc
tic
 % Perform the Kalman filtering, optimized
for k = 2:length(dt)
    % Predict
    Xk1 = X(1,k-1) + (u(k) - X(2,k-1))*dt(k);
    % Prediction error
    error = z(k) - Xk1;
    % Correct
    X(1,k) = Xk1 + K(1)*error;
    % Add to bias for making next prediction better
    X(2,k) = X(2,k-1) + K(2)*error;
end
toc
end

