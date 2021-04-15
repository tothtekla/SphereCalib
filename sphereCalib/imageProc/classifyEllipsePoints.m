function inlierIdxs = classifyEllipsePoints(points, ellipseParam, ransacThreshold)
%
% Select the indices of the ellipse inlier points
%
% points : 2D pointset, N by 2 matrix
% ellipsParams: parametric ellipse parameters in order: [a, b, ex, ey, theta]
% ransacThreshold: Euclidean distance error threshold of RANSAC
% inlierIdxs: final inlier indices
%   
inlierIdxs = zeros(length(points), 1);
numPts = 0;
d = zeros(numPts,1);

for i = 1 : length(points)
    % distance of the i-th point from sphere center with numerical approximation
    d(i) = pointEllipseDistance(points(i,1), points(i,2), ellipseParam(1), ...
        ellipseParam(2), ellipseParam(3), ellipseParam(4), ellipseParam(5));
    % if the point fits, save the index
    if(d(i) < ransacThreshold)
        numPts = numPts + 1;
        inlierIdxs(numPts) = i;
    end 
end
% save inlier indices
inlierIdxs = inlierIdxs(1:numPts);
end