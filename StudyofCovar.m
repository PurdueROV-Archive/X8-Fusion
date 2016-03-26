
Q = 10.^-(1:0.1:5);

for ii = 1:length(Q)
    K(:,ii) = Kalman1D_angle_covar(Q(ii),0.03,0.01,5000);
end

plot(K')