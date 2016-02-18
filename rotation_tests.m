% Do rotation in order of yaw-pitch-roll (z,y,x)
% That means from earth-sensor frame
% So going back from sensor to earth is x,y,z

% Refernce vector (pure gravity)
a = [0,0,1];

% Rotate reference
b = rotateZ(a, 1);
b = rotateY(a, 0.35);
b = rotateX(b, 0.35)

accelX = b(1);
accelY = b(2);
accelZ = b(3);

% Calculate the rotated angle
accelAngleX = atan2(-accelY, accelZ)
accelAngleY = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))


% Rotate back to check result
c = rotateX(b, -accelAngleX);
c = rotateY(c, -accelAngleY)
c = rotateZ(c, -1)




% For reference
% Simplified one
rx = atan2(-accelY, sqrt(accelX*accelX + accelZ*accelZ));
ry = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ));

dx = rotateX(b, -rx);
dy = rotateY(dx, -ry);

% Simplified two
r2x = atan2(-accelY, accelZ);
r2y = atan2(accelX, accelZ);

ex = rotateX(b, -r2x);
ey = rotateY(dx, -r2y);

