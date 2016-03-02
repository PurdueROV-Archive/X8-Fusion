data = load('easydata.csv');
u = data(:,2);
z = data(:,1);
dt = data(:,3);
K = [0.05716, 0.01694];
oldX = [0 0];
X = [0 0];
saveData = [0 0];
data2 = load('simpleoutput.txt');
for x = 1:100
    X(1) = (oldX(1) + oldX(2) + u(x) * dt(x) * dt(2) / 2);
    X(2) = (oldX(2) + u(x) * dt(x));
    error = z(x) - X(1);
    
    X = X + K * error;
    oldX = X;
    fprintf('X = %d, %d\n',oldX(1),oldX(2));
    %fprintf(fid,'X = %d, %d\n',oldX(1),oldX(2));
    
    saveData = [saveData; oldX];
end

figure(1);
subplot(2,1,1);
plot(linspace(1,100,length(saveData)),saveData);
subplot(2,1,2);
plot(linspace(1,100,length(data2)),data2);
hold on;
plot(linspace(1,100,length(data2)),data(:,1:2));