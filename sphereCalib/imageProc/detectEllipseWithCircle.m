function [inliers, ellipseImpl] = detectEllipseWithCircle(points, iterations, ransacThreshold, rMin, rMax)
%
% Detect ellipse points in 2D pointset using RANSAC
% Approximation with a circle
% Initial inlier set: 3 random points
%
% points : 2D pointset, N by 2 matrix
% iterations : no of RANSAC iterations
% ransacThreshold: Euclidean distance error threshold of RANSAC
% rMin, rMax: minimum and maximum value of circle radius for speed-up
% inliers: best fitted ellipse inlier points
% ellipseImpl : best fitted ellipse implicit parameters
%
numPts = length(points);
inlierIdxs = [];

for j = 1 : iterations
    % generate 3 random point 
    inlierIdxsTmp = randperm(numPts, 3);
    % approximate ellipse with circle parameters 
    [C0, r] = fitCircle3p(points(inlierIdxsTmp,:));
    % optional speed-up: a circle radius smaller than rMin or greater than rMax is assumed unrealistic
    if r > rMin && r < rMax
        % label points
        inlierIdxsTmp = classifyCirclePoints(points, C0, r, ransacThreshold);
        % save the best model
        if(length(inlierIdxsTmp) > length(inlierIdxs))
            inlierIdxs = inlierIdxsTmp;
        end
    end
end
% re-fit an ellipse
inliers = points(inlierIdxs, :);
ellipseImpl = fitEllipse5p(inliers);
end
