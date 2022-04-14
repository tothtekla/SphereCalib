function [rot, trans, inlierIdxs] = robustPointRegByRANSAC(pts1, pts2)
%
% Robust pointset registration with RANSAC
% The method filters the outliers of the pointsets and finds an accurate
% registration based on Arun et al., "Least-squares fitting of two
% 3-D pointsets."
% Note: applicable between any N-dimensional pointsets
%
% pts1:  first pointset, M by N matrix, M >= N
% pts2:  second pointset containing the transformed points of the first one
%        in the same order, M by N matrix
% rot:   rotation, N by N matrix 
% trans: translation, N element vector vector
% idxs: the indices of the points utilized in the registration
%
% The mapping :     pts2 = rot * pts1 + trans
% (no noise assumed)
%


iterations = 100000;
ransacThresholdReproj = 0.2;
minNoPts = 10;
numPts = length(pts1);
inlierIdxs = [];

for i = 1:iterations
    % generate 3 random point 
    minInlierIdxsTmp =  randperm(numPts, minNoPts);
    [rot, trans] = pointReg(pts1(minInlierIdxsTmp, :),pts2(minInlierIdxsTmp, :));
    % reproject pst1 with the trans. param.s
    pst1Reproj = (rot * pts1')' + trans;
    % error between the reprojected pst1 and pst2
    errorReproj = abs(vecnorm((pst1Reproj - pts2)')');
    % label points
    inlierIdxsTmp = find(errorReproj < ransacThresholdReproj);
    % save the best model
    if(length(inlierIdxsTmp) > length(inlierIdxs))
        inlierIdxs = inlierIdxsTmp;
        minInlierIdxs = minInlierIdxsTmp;
    end
end

[rot, trans] = pointReg(pts1(minInlierIdxs, :),pts2(minInlierIdxs, :));
 pst1Reproj = (rot * pts1')' + trans;
 errorReproj = abs(vecnorm((pst1Reproj - pts2)')');
 
end
   