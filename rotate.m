function [ Xr, MR] = rotate( Xg, Rx, Ry, Rz )
%Rotate Xu with the vector R (radians)
%   Xu: position in as global unit vector
%   Xr: resulting rotated vector
%   R: 3D rotation in radians.

% Rotation order yaw-pitch-roll (order of z, followed by y, followed by x)
% Tait-Bryan angles
% See http://danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part1.html


for k = 1:length(Rx)
    % Rotation matrix
    % Around X
    MX =    [1,   0,         0;
             0,   cos(Rx(k))   -sin(Rx(k));
             0,   sin(Rx(k))    cos(Rx(k))];

    % Around Y
    MY =    [cos(Ry(k)),   0,    sin(Ry(k));
             0,         1           0;
            -sin(Ry(k)),   0     cos(Ry(k))];

    % Around Z
    MZ =    [cos(Rz(k)),   -sin(Rz(k)),   0;
            sin(Rz(k)),   cos(Rz(k)),     0;
            0,          0,          1];

    MR = MZ*MY*MX;

    Xr(k,:) = MR*Xg(k,:)';
end

% If only one rotation defined
if(length(Rx) == 1)
    Xr = MR*Xg';
end

end

