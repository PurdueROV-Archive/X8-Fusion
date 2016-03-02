% Calculation of need thrust of desiered force vector
% F = M*T;
% F is a result from a PID or can for example be mapped from a 3D mouse.
% This makes it easy to drop or two motors as well, just remove it from the
% vectors...
%
% Test code for Purdue ROV x8
clear all;
% Desiered force vector
%function T = thurstmapping(F)
F = [50,0,0,0,0,0]';

% Pivot offset
pivoff = [0, 0, 0]'; % m
COM = [1.31, 0.0, 0.31]'; % inches
%pivoff = COM;
COM = [COM COM COM COM COM COM COM COM];

% Thruster locations: X Y Z in INCHES;
%Z is forward/back, y is up/down, X is left/right
loc =   [-4.7524, 0, 11.2148;
         4.7524, 0, 11.2148;
         -4.7524, 0, -7.631;
         4.7524, 0, -7.631;
         -5.5709, 5.6384, 6.0419;
         5.5709, 5.6384, 6.0419;
         -5.5709, 5.6384, -2.485; %]';
         5.5709, 5.6384, -2.485;];
     % Note the transpose! Its on everything in the code
     
%X is forward/back, Y is to the left/right, Z is up/down    
X = loc(:,3);
Y = loc(:,1);
Z = loc(:,2);


loc = [X Y Z]';

     
     
loc = loc - COM;

loc = loc * 2.54 / 100;


z = sqrt(1-.342^2);
% Thruster rotations components: X Y Z
rot =   [0.342 ,0, z;
         -0.342 ,0, z;
         0.342 ,0, -z;
         -0.342 ,0, -z;
         0,1,0;
         0,1,0;
         0,1,0; %]'; % uncomment to remove one motor
         0,1,0;];
     
X = rot(:,3);
Z = rot(:,2);
Y = rot(:,1);

rot = [X Y Z]';

     
% Calculate the new Force vector from the new pivot position
F(1:3) = F(1:3) + cross(pivoff,F(4:6));

% Build matrix
% Force

% Force
M = rot;  
% Force momentum
M(4:6,:) = cross(rot,loc,1);
%M(abs(M) < 0.01) = 0;
fid = fopen('results.txt','w');
Tmap = pinv(M);
Tmap(:,4:6) = Tmap(:,4:6)/10; % "Nm" is a lot stronger unit then N
val = round(Tmap*1024);


for y = 1:1:8
    fprintf(fid, ['\tmapper_matrices.matrices[0].t%d = vect6Make(' sprintf('%5d, ',val(y,1:5)) '%5d);\n'], y, val(y,6));
end
    fprintf(fid,'\n');


for x = 1:1:8
M = rot; 

% Force momentum
M(4:6,:) = cross(rot,loc,1);

%% is this correct? ask Jacob
M(:,x) = zeros([6 1]); %take the first motor "offline"


% Solve the linear equation system for minimum 2-norm on the thrust
% This spreads out the thrust on as many thruster as possible!
% Taking M\F will allways give two thruster unused.
Tmap = pinv(M);
Tmap(:,4:6) = Tmap(:,4:6)/10; % "Nm" is a lot stronger unit then N
val = round(Tmap*1024);

    for y = 1:1:8
        fprintf(fid, ['\tmapper_matrices.matrices[%d].t%d = vect6Make(' sprintf('%5d, ',val(y,1:5)) '%5d);\n'], x, y, val(y,6));
    end
    fprintf(fid,'\n');
end
%T = val*F
%end

fclose(fid);
T = val*F;
i2C = floor(1125.2*T);
%fprintf('%d\n',i2C);

Twanted = [32000;
    -32000;
    32000;
    -32000;
    0
    0
    0
    0]*748;

Fwanted = pinv(val)*Twanted;

