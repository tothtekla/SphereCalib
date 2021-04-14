function [inliers, S0] = detectSphereRand3p(points, r, iterations, ransacThreshold, aMin, aMax)
%
% Detect sphere projection (ellipse) contour points in 2D pointset using RANSAC
% Initial inlier set: 3 random points
% After then fitting sphere with the inliers
%
% points : 2D pointset, N by 2 matrix
% r: sphere radius
% iterations : no of RANSAC iterations
% ransacThreshold: Euclidean distance error threshold of RANSAC
% aMin, aMax: minimum and maximum value of longer ellipse semi-axis for speed-up
% inliers: best fitted ellipse inlier points
% S0 : best fitted sphere center
%
numPts = length(points);
inlierIdxs = [];

for j = 1 : iterations
    % generate 3 random point 
    inlierIdxsTmp = randperm(numPts, 3);
    % approximate sphere projection parameters with 3 points  
    ellipseParam = fitEllipse3p(points(inlierIdxsTmp,:));
    % optional speed-up: longer ellipse semi-axis smaller than aMin or greater than aMax is assumed unrealistic
    if ellipseParam(1) > aMin && ellipseParam(1) < aMax
        % label points
        inlierIdxsTmp = classifyEllipsePoints(points, ellipseParam, ransacThreshold);
        % save the best model
        if(length(inlierIdxsTmp) > length(inlierIdxs))
            inlierIdxs = inlierIdxsTmp;
        end
    end
end
% re-fit a sphere
inliers = points(inlierIdxs, :);
S0 = fitSphere3p(inliers, r);
end
