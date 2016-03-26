function [ K ] = Kalman1D_angle_covar( oQ, R, dt, N )
% Calculate the covariance matrix for a set of different variances on the
% variables. Return the converged Kalman gain K

% Example variances
% Q = 0.1;  % "Movement variation"
% R = 0.3162;     % "Location Measurement noise"


% Intinalize
F = [1, -dt;     % State transistion
     0, 1];

B = [dt ;
     0 ];
 
H = [1, 0]; % Observation model

 % Covariane martix
Q = oQ^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; 
P = eye(2).*Q;

for k = 2:N
    Pk = F*P*F' + Q;   % Covariance (size of the error) estimate
    S = H*Pk*H' + R;
    K = Pk*H'/S;
    P = (eye(2) - K*H)*Pk;
    %Kvec(:,k) = K;
end


end

