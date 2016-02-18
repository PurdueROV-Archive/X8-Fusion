function [ Xrot] = rotateZ( Xg, Rz )
%Rotate Xg around Z
%   Xg: position in as global unit vector
%   Xr: resulting rotated vector
%   Rz: Rotation in radians 
%   Both vectors assumed to be of same length


% Around X
    
Xr = cos(Rz)*Xg(1) -sin(Rz)*Xg(2); 
Yr = sin(Rz)*Xg(1) +cos(Rz)*Xg(2);

Xrot = [Xr, Yr, Xg(3)];

end

