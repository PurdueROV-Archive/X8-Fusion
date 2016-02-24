clear all

load('Logs/IMU_log_03.mat');

K = [0.01;
    -0.1];
 
 %Scale all variable to SI units
 DT = DT/1000000;   % DT is in us
 Ax = 4*9.81*Ax/32768;   % signed 16 bit 2g
 Ay = 4*9.81*Ay/32768;
 Az = 4*9.81*Az/32768;
 Gx = 17.4533*Gx/32768; % signed 16 bit radians/s
 Gy = 17.4533*Gy/32768;
 Gz = 17.4533*Gz/32768;
 Z = Z/1000; % Stored in mm
 
 
 [eRx, eRy] = acc2rot(Ax,Ay,Az);
 
 Rx = Kalman1D_IMU(Gx, eRx, DT, K);
 Ry = Kalman1D_IMU(Gy, eRy, DT, K);
 Rz = zeros(size(Rx)); % Assume no Z rotation at the moment
 
 cRx = compfilter(Gx, eRx, 0.02, mean(DT));
 cRy = compfilter(Gy, eRy, 0.02, mean(DT));
 
% Rotate acceleration from its local coordinates too global
% This means the reverse direction of the tilt of the system so -R
% A = rotate([Ax, Ay, Az], -Rx(1,:), -Ry(1,:), -Rz(1,:));
A = [Ax, Ay, Az];
A = rotateY(A, -Ry(1,:));
A = rotateX(A, -Rx(1,:));

 % Subtract gravitation
 A(:,3) = A(:,3) - 11;
 
 % A calibration hack here:
 %A(:,1) = A(:,1) - mean(A(100:500,1));
 %A(:,2) = A(:,2) - mean(A(100:500,2));
 %A(:,3) = A(:,3) - mean(A(100:500,3));
 
 
 K = [0.01;
    0.05];
 estZ = Kalman1D_IMU_loc( A(:,3), Z, DT, K );
 
 % Low pass reference and derivate as a reference
 Zs = LP_exp(Z, 0.97);
 vZ = [0, (Zs(2:end)-Zs(1:end-1))/2/mean(DT)];
 
 figure(1)
 subplot(2,1,1)
 plot([Rx', eRx])
 hold on
 plot(cRx(1,:), 'black');
 hold off;
 subplot(2,1,2)
 plot([Ry', eRy])
 hold on
 plot(cRy(1,:), 'black');
 hold off;
 
 figure(2)
 plot(A)
 
 figure(3)
 plot([estZ', Z, vZ', Zs', A(:,3)/10]);

 
 %fid = fopen('otherdata.txt','w');
 %fprintf(fid,'%f, %f\t %f, %f\t %f, %f\n',Rx(1,:), Rx(2,:), Ry(1,:), Ry(2,:), estZ(1,:), estZ(2,:))
 
