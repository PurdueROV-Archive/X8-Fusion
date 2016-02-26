function [ Rx, Ry ] = acc2rot( Ax, Ay, Az )
% Rotation estimate from accelrometer data
%   Can't compute Rz, need Mag data for that.


%Rx = atan(Ay./sqrt(Az.^2 + Ax.^2));
%Ry = atan(Ax./sqrt(Az.^2 + Ay.^2));

% Correct method
% Calculate the rotated angle
Ry = atan2(-Ax, Az);
Rx = atan2(Ay, sqrt(Ay.*Ay + Az.*Az));


end

