function [a, b, ex, ey, theta] = implEllipse2paramEllipse(A, B, C, D, E, F)
%
% Conversion from implicit ellipse
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%            to parametric ellipse  
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey 
%
Disc = B^2 - 4*A*C;
a = (-sqrt(2*(A*E^2 + C*D^2 - B*D*E + Disc*F)*((A + C) + sqrt((A - C)^2 + B^2)))) / Disc;
b = (-sqrt(2*(A*E^2 + C*D^2 - B*D*E + Disc*F)*((A + C) - sqrt((A - C)^2 + B^2)))) / Disc;
ex = (2*C*D - B*E) / Disc;
ey = (2*A*E - B*D) / Disc;
if (B == 0)
    if (A < C)
        theta = 0;
    else
        theta = 0.5 * pi;
    end
else
    theta = atan(1/B * (C - A - sqrt((A - C)^2 + B^2)));
end
end