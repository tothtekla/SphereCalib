function [inliers, S0, r] = detectSphereRand4p(points, iterations, ransacThreshold, rMin, rMax)
%
% Detect sphere surface points in 3D point cloud using RANSAC
% Initial inlier set: 4 random points
%
% points : 3D point cloud, N by 3 matrix
% iterations : no. of RANSAC iterations
% ransacThreshold: Euclidean distance error threshold of RANSAC
% rMin, rMax: minimum and maximum value of sphere radius for speed-up
% inliers: best fitted sphere inlier points
% S0: best fitted sphere center
% r:  best fitted sphere radius
%
numPts=length(points);
inlierIdxs = [];

for i = 1 : iterations
    % generate 4 random point
    inlierIdxsTmp = randperm(numPts, 4);
    % find sphere parameters
    [S0, r] = fitSphereNonit(points(inlierIdxsTmp, 1 ), points(inlierIdxsTmp, 2), points(inlierIdxsTmp, 3));
    % optional speed-up: a sphere radius smaller than rMin or greater than rMax is assumed unrealistic
    if r > rMin && r < rMax
        % label points
        inlierIdxsTmp = classifySpherePoints(points, S0, r, ransacThreshold);       
        % save the best model
        if(length(inlierIdxsTmp) > length(inlierIdxs))
            inlierIdxs = inlierIdxsTmp;
        end
    end
end
% re-fit
inliers = points(inlierIdxs,:);
[S0, r] = fitSphereLsq(inliers);
end