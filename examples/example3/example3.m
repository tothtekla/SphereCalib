function [rotEst, transEst, rEst] = example3
%
% Calibration example on real data
% The radius of the calibration sphere was around 0.3 meter
%
%{
load('input3C1.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0');
%}
%%{
load('input3C2.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0');
%}

lidPlanes = 4;    
lidRansac = 1;
edgeDetect = 1;
edgeThreshold = 0.3;
camRansac = 2;
showFigures = true;

[imgs, points] = readFiles(imgDir, imgNames, scan3dDir, scan3dNames);
pointsAll = points;
tic;
points = deletePlanes(points, lidPlanes, showFigures);
[rotEst, transEst, S0CamEst, S0LidEst, rEst, sphInliers] = sphereCalib(imgs, points, fu, fv, u0, v0,...
    lidRansac, [], edgeDetect, edgeThreshold, camRansac, showFigures);
toc;
transEst2 = mean(S0LidEst)-mean((rotEst*S0CamEst')'); %#ok<UDIM>
for i = 1:length(imgNames)    
    fileNameBP = strcat('examples/example3/outputC2camRansac2/sphereCalibBackproj', num2str(i)); 
    fileNameCP = strcat('examples/example3/outputC2camRansac2/sphereCalibScan', num2str(i)); 
    plotBackProjection(imgs{i}, sphInliers{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameBP);
    plotColoredProjection(imgs{i}, pointsAll{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameCP);
end
end