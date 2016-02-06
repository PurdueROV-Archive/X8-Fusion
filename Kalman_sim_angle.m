clear all

find_optimal = 0;
complimentary = 0;

dt = 0.1;      % Sampling delay
t = 0:dt:100;    % time
holdN = 1;      % hold reference for # accelrometer samples

% Kalman gains
Q = 0.1;  % "Movement variation"
R = 0.3162;     % "Location Measurement noise"


% Intinalize
F = [1, -dt;     % State transistion
     0, 1];

B = [dt ;
     0 ];
 
H = [1, 0]; % Observation model

% Ground thurth
v = 10*cos(pi*t/10)+20*cos(pi*t);     % Speed
loc = [cumsum(v*dt);     % Location
     v];    % Speed

% Measurments (added noise)
u = v + 50*(rand(1,length(v))-0.5);
z = loc(1,:) + 400*(rand(1,length(v))-0.5);

% Sample and hold z
z = sampleNhold(z,holdN);

figure(5)
subplot(2,1,1)
plot(z)
title('Angle measured')

subplot(2,1,2)
plot(u)
title('Speed measured')

if(find_optimal)
    
    errorL = zeros(16);
    errorV = zeros(16);
    for i = 1:16;
        for j = 1:16;
            Q = 10^-(i/2);
            R = 10*10^-(j/2-0.5);
            X = Kalman1D_angle( Q, R, F, B, u, z, H, dt, t);
            errorL(i,j) = sum([loc(1,500:end)-X(1,500:end)]'.^2);
            errorV(i,j) = sum([loc(2,500:end)-X(2,500:end)]'.^2);
        end
    end
    
    figure(2)
    surf(log(errorL));
    title('Location error');
    figure(3)
    surf(log(errorV));
    title('Velocity error');
    
    % Find optimal gains
    [minerrK,ind] = min(errorL(:));
    minerrK
    minerrKV = errorV(ind)
    [m,n] = ind2sub(size(errorL),ind);

    Q = 10^-(m/2);
    R = 10*10^-(n/2-0.5);
end

if(complimentary)
    fact = 0:0.01:1;
    errorCL = zeros(1,length(fact));
    errorCV = errorCL;
    
    for i = 1:length(fact);
        X = compfilter(u, z, fact(i), dt);

        errorCL(i) = sum([loc(1,:)-X(1,:)]'.^2);
        errorCV(i) = sum([loc(2,:)-X(2,:)]'.^2);
    end
    
    figure(4)
    plot(fact,[log(errorCL); log(errorCV)]');
    title('Complimentary filter error')
    [minerrC,ind] = min(errorCL);
    minlocC = errorCL(ind)
    minvelC = errorCV(ind)
    
    Xc = compfilter(u, z, fact(ind), dt);
end
    



X = Kalman1D_angle( Q, R, F, B, u, z, H, dt, t);

figure(1)
subplot(2,1,1)
plot([loc(1,:);X(1,:)]')
hold on;
plot(z,'+');
hold off;
title('Position');
if(complimentary)
    hold on;
    plot(Xc(1,:)', 'black')
    hold off;
end

subplot(2,1,2)
plot([loc(1,:)-X(1,:)]');
title('Error')

if(complimentary)
    hold on;
    plot(loc(1,:)-Xc(1,:), 'black');
    hold off;
end

