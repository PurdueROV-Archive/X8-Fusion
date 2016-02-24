clear
load Logs/easydata.csv
load C/simpleoutput.txt

z = easydata(:,1);
u = easydata(:,2);
K = [0.05716, 0.01694];
X = zeros(2,101);
X(1,1) = z(1);
X(1,2) = u(2);
mdt = 0.1;
for k = 2:101
Xk1 = X(1,k-1) + X(2,k-1)*mdt + mdt^2/2 *u(k);
Xk2 = X(2,k-1) + u(k)*mdt;
% Prediction error
error = z(k) - Xk1;
% Correct
X(1,k) = Xk1 + K(1)*error;
% Add to bias for making next prediction better
X(2,k) = Xk2 + K(2)*error;
end

close all;
subplot(2,1,1);
plot(simpleoutput(:,1),'r');
hold on;
plot(simpleoutput(:,2),'b');
hold off;
subplot(2,1,2);
plot(X(:,1),'r');
hold on;
plot(X(:,2),'b');
hold off;
