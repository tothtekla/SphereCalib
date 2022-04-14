function [p, n] = fitPlane3p(points)
%
% Minimal sphere pőlane algorithm with 3 points
%
% points: plane surface points, 3 by 3 matrix
% p: plane point
% n: normal vector
%
p = mean(points);
v1 = points(2, :) - points(1,:);
v2 = points(3, :) - points(1,:); 
n = cross(v1, v2);
if isnan(n) | isinf(n) | ~norm(n) %#ok<OR2>
    n = [1 0 0];
else
    n = n / norm(n);
end
end

