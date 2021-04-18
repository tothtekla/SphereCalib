function inlierIdxs = classifySpherePoints(points, S0, r, ransacThreshold)
%
% Select the indices of the sphere inlier points
%
% points : 3D point cloud, N by 3 matrix
% S0: best fitted sphere center
% r:  best fitted sphere radius
% ransacThreshold: Euclidean distance error threshold of RANSAC
% inlierIdxs: final inlier indices
%

% point distance from the sphere center
radii = S0-points;
radii = sqrt(sum(radii.*radii,2));
% radius error
distances = abs(r - radii);
% if the point fits, save the index
inlierIdxs = find(distances < ransacThreshold);
end