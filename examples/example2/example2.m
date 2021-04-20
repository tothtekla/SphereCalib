function [rotEst, transEst, rEst] = example2
%
% Calibration example on real data
% The radius of the calibration sphere was around 0.3 meter
%
load('input2.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0');

lidPlanes = 5;
closeThreshold = 1.5; % <- for L1 scans, L2 scans: closeThreshold = 1.2
lidRansac = 1;
edgeDetect = 1;
edgeThreshold = 0.25;
camRansac = 2;
showFigures = true;

[imgs, points] = readFiles(imgDir, imgNames, scan3dDir, scan3dNames);
pointsAll = points;
tic;
points = deletePlanes(points, lidPlanes, closeThreshold, showFigures);
[rotEst, transEst, S0CamEst, S0LidEst, rEst, sphInliers] = sphereCalib(imgs, points, fu, fv, u0, v0,...
    lidRansac, [], edgeDetect, edgeThreshold, camRansac, showFigures);
toc;
transEst2 = mean(S0LidEst)-mean((rotEst*S0CamEst')'); %#ok<UDIM>
for i = 1:length(imgNames)    
    fileNameBP = strcat('examples/example2/output/sphereCalibBackproj', num2str(i)); 
    fileNameCP = strcat('examples/example2/output/sphereCalibScan', num2str(i)); 
    plotBackProjection(imgs{i}, sphInliers{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameBP);
    plotColoredProjection(imgs{i}, pointsAll{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameCP);
end
end