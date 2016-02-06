function [ Xrot, MR] = rotate2( Xg, Rx, Ry, Rz )
%Rotate Xu with the vector R (radians)
%   Xu: position in as global unit vector
%   Xr: resulting rotated vector
%   R: 3D rotation in radians.

k = 1;
% Around X
MX =    [1,   0,         0;
         0,   cos(Rx(k)),   -sin(Rx(k));
         0,   sin(Rx(k)),    cos(Rx(k))];

Yr = cos(Rx(k))*Xg(2) -sin(Rx(k))*Xg(3); 
Zr = sin(Rx(k))*Xg(2) +cos(Rx(k))*Xg(3);

% Around Y
MY =    [cos(Ry(k)),   0,    sin(Ry(k));
         0,         1           0;
        -sin(Ry(k)),   0     cos(Ry(k))];
    
Xr =  cos(Ry(k))*Xg(1) +sin(Ry(k))*Zr; 
Zr = -sin(Ry(k))*Xg(1) +cos(Ry(k))*Zr;

% Around Z
MZ =    [cos(Rz(k)),   -sin(Rz(k)),   0;
        sin(Rz(k)),   cos(Rz(k)),     0;
        0,          0,          1];
    
Xr = cos(Rz(k))*Xr -sin(Rz(k))*Yr; 
Yr = sin(Rz(k))*Xr +cos(Rz(k))*Yr;

    MR = MZ*MY*MX;
    Xrot = MR*Xg'
    [Xr; Yr; Zr]
    

end

