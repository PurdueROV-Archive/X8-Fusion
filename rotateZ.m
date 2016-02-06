function [ Xrot] = rotateZ( Xg, Rz )
%Rotate Xg around Z
%   Xg: position in as global unit vector
%   Xr: resulting rotated vector
%   Rz: Rotation in radians 

k = 1;
% Around X
    
Xr = cos(Rz(k))*Xg(1) -sin(Rz(k))*Xg(2); 
Yr = sin(Rz(k))*Xg(1) +cos(Rz(k))*Xg(2);

Xrot = [Xr, Yr, Xg(3)];

end

