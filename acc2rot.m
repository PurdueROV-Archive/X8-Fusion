function [ Rx, Ry ] = acc2rot( Ax, Ay, Az )
% Rotation estimate from accelrometer data
%   Can't compute Rz, need Mag data for that.

Rx = atan(Ay./Az);
Ry = atan(Ax./Az);



end

