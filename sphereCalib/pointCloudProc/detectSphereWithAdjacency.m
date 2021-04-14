function [inliers, S0, r] = detectSphereWithAdjacency(points, iterations, adjacencyThreshold, ransacThreshold, rMin, rMax)
% 
% Detect sphere surface points in 3D point cloud using RANSAC
% Initial inlier set: 1 random point + k-NN where k = adjacencyThreshold
%
% Based on Tekla Tóth, Levente Hajder: "Robust Fitting of Geometric Primitives on LiDAR Data"
%
% points : 3D point cloud, N by 3 matrix
% iterations : no. of RANSAC iterations
% adjacencyThreshold : select the closest subset of point withtis size
% ransacThreshold: Euclidean distance error threshold of RANSAC
% rMin, rMax: minimum and maximum value of sphere radius for speed-up
% inliers: best fitted sphere inlier points
% S0: best fitted sphere center
% r:  best fitted sphere radius
%
numPts=length(points);
inlierIdxs = [];

%data structures for knn-search
MdlKDT = KDTreeSearcher(points);

for i = 1 : iterations
    % generate 1 random point
    randIdx = randi(numPts); 
    % select initial subset
    inlierIdxsTmp = knnsearch(MdlKDT, points(randIdx, :), 'K', adjacencyThreshold);
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
inliers = points(inlierIdxs, :);
[S0, r] = fitSphereLsq(inliers);
end