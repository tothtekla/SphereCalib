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
inlierIdxs = zeros(length(points), 1);
numPts = 0;
for i = 1 : length(points)
    % distance of the i-th point from sphere center
    rTmp = norm(S0-points(i, :));
    d = abs(r - rTmp);
    % save the index if the point fits
    if(d < ransacThreshold)
        numPts = numPts + 1;
        inlierIdxs(numPts) = i;
    end
end
% save inlier indices
inlierIdxs = inlierIdxs(1:numPts);
end