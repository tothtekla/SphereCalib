function paramEllipse = implEllipseVec2paramEllipseVec(implEllipse)
%
% Conversion from implicit ellipse
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%            to parametric ellipse  
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey 
%
[a, b, ex, ey, teta] =  implEllipse2paramEllipse(implEllipse(1), implEllipse(2), implEllipse(3), implEllipse(4), implEllipse(5), implEllipse(6));
paramEllipse = [a, b, ex, ey, teta];
end