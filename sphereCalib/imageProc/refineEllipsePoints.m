function [inliers, S0, ellipseParam] = refineEllipsePoints(inliers_ini, points, r, ransacThreshold)
%
% Refine sphere projection (ellipse) contour points in a more detailed 2D pointset 
%
% inliers_ini : initial inlier set
% points : 2D pointset, N by 2 matrix
% r: sphere radius
% ransacThreshold: Euclidean distance error threshold of RANSAC
% inliers: best fitted ellipse inlier points
% S0 : best fitted sphere center
%

% approximate sphere projection parameters with 3 points  
ellipseParam = fitEllipse3p(inliers_ini);

% label points
inlierIdxs = classifyEllipsePoints(points, ellipseParam, ransacThreshold);

% re-fit an ellipse and a sphere
inliers = points(inlierIdxs, :);
ellipseParam = fitEllipse3p(inliers);
S0 = fitSphere3p(inliers, r);