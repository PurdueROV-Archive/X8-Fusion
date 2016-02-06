function [ Xrot] = rotateX( Xg, Rx)
%Rotate Xg around X
%   Xg: position in as global unit vector
%   Xr: resulting rotated vector
%   Rx: Rotation in radians 

k = 1;
% Around X

Yr = cos(Rx(k))*Xg(2) -sin(Rx(k))*Xg(3); 
Zr = sin(Rx(k))*Xg(2) +cos(Rx(k))*Xg(3);

Xrot = [Xg(1), Yr, Zr];

end

