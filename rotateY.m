function [ Xrot] = rotateY( Xg, Ry )
%Rotate Xg around Y
%   Xg: position in as global unit vector
%   Xr: resulting rotated vector
%   Ry: Rotation in radians 
%   Both vectors assumed to be of same length



% Around Y
    
Xr =  cos(Ry').*Xg(:,1) +sin(Ry').*Xg(:,3); 
Zr = -sin(Ry').*Xg(:,1) +cos(Ry').*Xg(:,3);

Xrot = [Xr, Xg(:,2), Zr];

end

