function [ Xrot] = rotateX( Xg, Rx)
%Rotate Xg around X
%   Xg: position in as global unit vector
%   Xr: resulting rotated vector
%   Rx: Rotation in radians 
%   Both vectors assumed to be of same length

% Around X

Yr = cos(Rx').*Xg(:,2) -sin(Rx').*Xg(:,3); 
Zr = sin(Rx').*Xg(:,2) +cos(Rx').*Xg(:,3);

Xrot = [Xg(:,1), Yr, Zr];

end

