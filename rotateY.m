function [ Xrot] = rotateY( Xg, Ry )
%Rotate Xg around Y
%   Xg: position in as global unit vector
%   Xr: resulting rotated vector
%   Ry: Rotation in radians 

k = 1;

% Around Y
    
Xr =  cos(Ry(k))*Xg(1) +sin(Ry(k))*Xg(3); 
Zr = -sin(Ry(k))*Xg(1) +cos(Ry(k))*Xg(3);

Xrot = [Xr, Xg(3), Zr];

end

