function inlierIdxs = classifyCirclePoints(points, C0, r, ransacThreshold)
%
% Select the indices of the circle inlier points
%
% points : 2D pointset, N by 2 matrix
% C0: best fitted circle center
% r:  best fitted sphere radius
% ransacThreshold: Euclidean distance error threshold of RANSAC
% inlierIdxs: final inlier indices
%

% point distance from the circle center
radii =  C0-points;
radii = sqrt(sum(radii.*radii, 2));
% radius error
distances = abs(r - radii);
% if the point fits, save the index
inlierIdxs = find(distances < ransacThreshold);
end