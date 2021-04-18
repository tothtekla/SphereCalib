function inlierIdxs = classifyPlanePoints(points, p, n, ransacThreshold)
%
% Select the indices of the plane inlier points
%
% points : 3D point cloud, N by 3 matrix
% p: plane point
% n: normal vector
% ransacThreshold: Euclidean distance error threshold of RANSAC
% inlierIdxs: final inlier indices
%

% distance of the points from the plane
distances = abs(sum(n.*(points-p), 2));
% if the point fits, save the index
inlierIdxs = find(distances < ransacThreshold);
end