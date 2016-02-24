clear
clc
close all

fid1 = fopen('otherdata.txt','r');
fid2 = fopen('C/output.txt','r');

data1 = fscanf(fid1,'%f, %f\t %f, %f\t %f, %f\n',[6 5000]);
data2 = fscanf(fid2,'%f, %f\t %f, %f\t %f, %f\n',[6,5000]);
data1 = data1';
data2 = data2';


figure(1);
plot(data1(:,5),'red');

fprintf('%f\t%f',min(data1(:,1)), max(data1(:,1)))
hold on;
plot(data2(:,1),'blue');
hold off;
