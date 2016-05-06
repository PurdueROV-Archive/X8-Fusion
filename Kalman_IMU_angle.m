clear all

load('Logs/attitude02.mat');
% ONLY for attitue02.mat
m = (Gx(2172)+Gx(2176))/2
Gx(2173:2175) = Gx(2173:2175) +m-mean(Gx(2173:2175));

find_optimal = 0;

K = [0.005;
    -0.00];
 
 %Scale all variable to SI units
 DT = ones(length(Ax),1)/100;
 %DT = DT/1000000;   % DT is in us
 Ax = 4*9.81*Ax/32768;   % signed 16 bit 2g
 Ay = 4*9.81*Ay/32768;
 Az = 4*9.81*Az/32768;
 %Gx = 17.4533*Gx/32768; % signed 16 bit radians/s
 %Gy = 17.4533*Gy/32768;
 %Gz = 17.4533*Gz/32768;
 Gx = 2*pi/180*Gx/10000;
 Gy = 2*pi/180*Gy/10000;
 Gz = 2*pi/180*Gz/10000;
 
 [eRx, eRy] = acc2rot(Ax,Ay,Az);
 
 
 if(find_optimal)
    
    errorL = zeros(16);
    errorV = zeros(16);
    for i = 1:16;
        for j = 1:16;
            K(1) = 5*10^-(i/4);
            K(2) = -10*10^-(j/4-0.25);
            X = Kalman1D_IMU(Gx, eRx, DT, K);
            errorL(i,j) = sum([-9.6338e-04-X(1,50:end)]'.^2);
        end
    end
    
    figure(10)
    surf(log(errorL));
    title('Location error');
    
    % Find optimal gains
    [minerrK,ind] = min(errorL(:));
    minerrK
    [m,n] = ind2sub(size(errorL),ind);

    K(1) = 5*10^-(m/4);
    K(2) = 10*10^-(n/4-0.25);
 end

 
 [Rx, Kd] = Kalman1D_IMU_dyn(Gx, eRx, DT, K);
 Ry = Kalman1D_IMU(Gy, eRy, DT, K);
 
 cRx = compfilter(Gx, eRx, 0.05, mean(DT));
 cRx2 = compfilter(Gx, eRx, 0.00001, mean(DT));
 cRy = compfilter(Gy, eRy, 0.00001, mean(DT));
 
 
 figure(1)
 title('angles')
 %subplot(2,1,1)
 plot([Rx(1,:)',10*Rx(2,:)', eRx])
 hold on
 plot([cRx(1,:)',cRx2(1,:)']);
 hold off;
 %subplot(2,1,2)
%  figure(2)
%  plot([Ry', eRy])
%  hold on
%  plot(cRy(1,:), 'black');
%  hold off;
 
 %figure(3)
 %plot([Rx(1,:)'-eRx, Ry(1,:)'-eRy])
 %title('Error')
 figure(4)
 plot([Kd(1,:); Kd(2,:)]')
 