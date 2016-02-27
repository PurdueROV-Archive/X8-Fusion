function [ X ] = Kalman1D_IMU( u, z, dt, K )
% 1D rotational Kalman filtering of IMU data
%   u = Gyro
%   z = estimated rotation from Acc and Mag data
%   K = Pre computed Kalman gain

 % Estimated state
X = [z(1);     % Alpha (angle)
     0];    % Bias

 % Perform the Kalman filtering, optimized
for k = 2:length(dt)
    % Predict
    Xk1 = X(1,k-1) + (u(k) - X(2,k-1))*dt(k);
    % Prediction error
    error = z(k) - Xk1;
    
    % Dynamic filter strength
    K(1) = K(1)*0.99 + 0.01*(0.001+0.01*u(k));
    
    % Correct
    X(1,k) = Xk1 + K(1)*error;
    % Add to bias for making next prediction better
    X(2,k) = X(2,k-1) + K(2)*error;
end

end

