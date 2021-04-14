function dist = pointEllipseDistance(px, py, a, b, ex, ey, theta)
%
% Point-ellipse distance with numerical approximation
%
% [px, py] : 2D point
% [a, b, ex, ey, theta] : ellipse parameters
% dist : estimated distance
%
ab = [a b];
px = cos(theta)*(px - ex) + sin(theta)*(py - ey);
py = - sin(theta)*(px - ex)  + cos(theta)*(py - ey);
p = [abs(px)  abs(py)];
k0 = norm(p/ab);
k1 = norm(p/(ab.*ab));
dist = abs(k0*(k0-1)/k1); 
end