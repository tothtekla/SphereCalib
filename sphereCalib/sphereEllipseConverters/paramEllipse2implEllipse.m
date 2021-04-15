function [A, B, C, D, E, F] = paramEllipse2implEllipse(a, b, ex, ey, theta)
%
% Conversion from parametric ellipse
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey
%            to implicit ellipse 
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%
A = a^2 * sin(theta)^2 + b^2 * cos(theta)^2;
B = 2*(b^2 - a^2) * sin(theta) * cos(theta);
C = a^2*cos(theta)^2 + b^2*sin(theta)^2;
D = -2*A*ex - B*ey ;
E = -B*ex - 2*C*ey;
F = A*ex^2 + B*ex*ey + C*ey^2 - a^2*b^2;
end