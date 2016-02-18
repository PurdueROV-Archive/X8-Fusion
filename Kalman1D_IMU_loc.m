function [ Xd ] = Kalman1D_IMU_loc( u, z, dt, K )
% 1D rotational Kalman filtering of IMU data
%   u = Accelrometer without gravity and in global coordinates
%   z = Measured position (Ultrasonic distance, GPS etc)
%   K = Pre computed Kalman gain

% Estimated state
X = [z(1);     % Alpha (angle)
     0];    % Bias
Xd = X; 
mdt = mean(dt);

% Intinalize
F = [1, mdt;     % State transistion
     0, 1];

B = [mdt^2 /2;
     mdt ];
 
H = [1, 0]; % Observation model

tic
for k = 2:length(dt)
    % Predict
    Xk = F*Xd(:,k-1) + B*u(k); % State estimate
    
    % Update
    e = z(k) - H*Xk; % measurement residual (error)
    Xd(:,k) = Xk + K*e;
end
toc
tic
 % Perform the Kalman filtering, optimized
for k = 2:length(dt)
    % Predict
    Xk1 = X(1,k-1) + X(2,k-1)*mdt + mdt^2/2 *u(k);
    Xk2 = X(2,k-1) + u(k)*mdt;
    % Prediction error
    error = z(k) - Xk1;
    % Correct
    X(1,k) = Xk1 + K(1)*error;
    % Add to bias for making next prediction better
    X(2,k) = Xk2 + K(2)*error;
end
toc


end

