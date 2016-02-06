function [ y ] = LP_exp( x, fac )
%Low pass filter with an moving exponential filter
%  fac = 0.67 looks good.
%  equals (2*y(k-1) + x(k))/3

y(1) = x(1);

for k = 2:length(x)
    
    y(k) = y(k-1)*fac + x(k)*(1-fac);
    
end 

end

