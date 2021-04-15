function paramEllipse = implSphereVec2paramEllipseVec(implSphere)
%
% Conversion from implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
%            to projected parametric ellipse     
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey  
%
[a, b, ex, ey, theta] =  implSphere2paramEllipse(implSphere(1), implSphere(2), implSphere(3), implSphere(4));
paramEllipse = [a, b, ex, ey, theta];
end