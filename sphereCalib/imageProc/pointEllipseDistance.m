function dist = pointEllipseDistance(px, py, a, b, ex, ey, theta)
%
% Point-ellipse distance with numerical approximation
% Based on: https://www.iquilezles.org/www/articles/ellipsoids/ellipsoids.htm
%
% [px, py] : 2D point
% [a, b, ex, ey, theta] : ellipse parameters
% dist : estimated distance
%
ab = [a b];
px0 = cos(theta)*(px - ex) + sin(theta)*(py - ey);
py0 = - sin(theta)*(px - ex)  + cos(theta)*(py - ey);
p = [abs(px0)  abs(py0)];
k0 = norm(p./ab);
k1 = norm(p./(ab.*ab));
dist = abs(k0*(k0-1)/k1); 
end