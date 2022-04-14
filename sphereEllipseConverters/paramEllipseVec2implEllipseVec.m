function implEllipse = paramEllipseVec2implEllipseVec(paramEllipse)
%
% Conversion from parametric ellipse
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey
%            to implicit ellipse 
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%
[A, B, C, D, E, F] = paramEllipse2implEllipse(paramEllipse(1), paramEllipse(2), paramEllipse(3), paramEllipse(4), paramEllipse(5));
implEllipse = [A, B, C, D, E, F];
end