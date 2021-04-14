function [S0LidMeanErr, S0LidStd, S0CamMeanErr, S0CamStd, rErr, transErr, rotErr] = example1
%
% Calibration example on synthetic data
%
load('input1.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0', 'rGT', 'S0CamGT', 'S0LidGT', 'rotGT', 'transGT');

lidRansac = 1;
edgeDetect = 2;
camRansac = 1;
showFigures = true;

[imgs, points] = readFiles(imgDir, imgNames, scan3dDir, scan3dNames);

tic;
[rotEst, ~, S0CamEst, S0LidEst, rEst] = sphereCalib(imgs, points, fu, fv, u0, v0, lidRansac, edgeDetect, camRansac, showFigures);
toc;

[S0LidMeanErr, S0LidStd] = estimateSphereCenterError(S0LidEst, S0LidGT);
[S0CamMeanErr, S0CamStd] = estimateSphereCenterError(S0CamEst, S0CamGT);
rErr = abs(rGT- rEst);
% Note: the stored GT transformation (rotate then translate) order is 
% opposite like in the pointset registration method (translate then rotate).
% Hence, translation has to be re-estimated. 
transEst = mean(S0LidEst)-mean((rotEst*S0CamEst')'); %#ok<UDIM>
[transErr, rotErr] = estimateTransformError(transEst, rotEst, transGT, rotGT);

for i = 1:length(imgNames)
    plotBackProjection(imgs{i}, points{i}, rotGT, transGT, fu, fv, u0, v0);
    plotColoredProjection(imgs{i}, points{i}, rotGT, transGT, fu, fv, u0, v0);
end
for i = 1:length(imgNames)
    plotBackProjection(imgs{i}, points{i}, rotEst, transEst, fu, fv, u0, v0);
    plotColoredProjection(imgs{i}, points{i}, rotEst, transEst, fu, fv, u0, v0);
end
end