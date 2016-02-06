function [ y ] = sampleNhold( x, N )
%Sample and holds every N sample, doesn't change vector length
%   x = full sample vector
%   N = hold sample for k = N

% Last numbers that aren't dividable.
modds = mod(length(x),N);

num = length(x) - modds;
y = zeros(size(x));

for k = 1:N
    y(k:N:num-N+k) = x(1:N:num-N+1);
end

if(modds)
    y(num+1:end) = x(num+1);
end
end

