function [S0LidMeanErr, S0LidStd, S0CamMeanErr, S0CamStd, rErr, transErr, rotErr] = example1
%
% Calibration example on synthetic data
%
load('input1.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0', 'rGT', 'S0CamGT', 'S0LidGT', 'rotGT', 'transGT');
lidRansac = 1;
camRansac = 1;
showFigures = true;
tic;
[rotEst, ~, S0CamEst, S0LidEst, rEst] = sphereCalib(imgDir, imgNames, scan3dDir,...
    scan3dNames, fu, fv, u0, v0, lidRansac, camRansac, showFigures);
toc;
[S0LidMeanErr, S0LidStd] = estimateSphereCenterError(S0LidEst, S0LidGT);
[S0CamMeanErr, S0CamStd] = estimateSphereCenterError(S0CamEst, S0CamGT);
rErr = abs(rGT- rEst);
transEst = mean(S0LidEst)-mean((rotEst*S0CamEst')'); %#ok<UDIM>
[transErr, rotErr] = estimateTransformError(transEst, rotEst, transGT, rotGT);
end