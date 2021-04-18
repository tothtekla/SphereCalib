function inlierIdxs = classifyEllipsePoints(points, ellipseParam, ransacThreshold)
%
% Select the indices of the ellipse inlier points
%
% points : 2D pointset, N by 2 matrix
% ellipsParams: parametric ellipse parameters in order: [a, b, ex, ey, theta]
% ransacThreshold: Euclidean distance error threshold of RANSAC
% inlierIdxs: final inlier indices
%   

% distance of the points from the ellipse with numerical approximation
distances = pointEllipseDistance(points(:,1), points(:,2), ellipseParam(1), ...
    ellipseParam(2), ellipseParam(3), ellipseParam(4), ellipseParam(5));
% if the point fits, save the index
inlierIdxs = find(distances < ransacThreshold);
end