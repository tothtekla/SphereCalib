function [a, b, ex, ey, theta] = implSphere2paramEllipse(x0, y0, z0, r)
%
% Conversion from implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
%            to projected parametric ellipse     
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey  
%
a = r*sqrt(x0^2 + y0^2 + z0^2 - r^2)/(z0^2 - r^2);
b = r/sqrt(z0^2 - r^2);
ex = x0*z0/(z0^2 - r^2);
ey = y0*z0/(z0^2 - r^2);
if y0 == 0
    theta = 0;
elseif x0 == 0
    theta = 0.5 * pi;
else
    theta = atan(y0/x0);
end  
end