function [S0MeanErr, S0Std] = estimateSphereCenterError(S0Est, S0GT)
%
% Estimate the error of the sphere center estimation
%
% S0Est : estimated sphere centers, N by 3 matrix
% S0GT :  ground truth sphere centers, N by 3 matrix
% S0MeanErr : mean error
% S0Std : standard deviation
%
S0Norm = vecnorm((S0GT-S0Est)');
S0MeanErr = mean(S0Norm);
S0Std = std(S0Norm);
end