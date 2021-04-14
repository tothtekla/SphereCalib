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
inlierIdxs = zeros(length(points), 1);
numPts = 0;
for i = 1 : length(points)
	% actual distance from ellipse
    rTmp = norm(C0-points(i, :));
    d = abs(r - rTmp);
    % if the point fits, save the index
    if(d < ransacThreshold)
        numPts = numPts + 1;
        inlierIdxs(numPts) = i;
    end
end
% save inlier indices
inlierIdxs = inlierIdxs(1:numPts);
end