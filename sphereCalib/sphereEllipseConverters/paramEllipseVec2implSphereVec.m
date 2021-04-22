function  sphereCenter = paramEllipseVec2implSphereVec(paramEllipseVec, r)
%
% Conversion from projected parametric ellipse        
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey  
%            to implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
% 
[x0, y0, z0] = fit3pSphereWithParamEllipse(paramEllipseVec(1),...
    paramEllipseVec(2), paramEllipseVec(3), paramEllipseVec(4),...
    paramEllipseVec(5), r);
sphereCenter = [x0 y0 z0];
end