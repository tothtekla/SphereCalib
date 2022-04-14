function [rotEst, transEst, rEst] = helloHajder
%
% Calibration example on real data
% The radius of the calibration sphere was around 0.3 meter
%
%{
load('hexample.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0');
%}
%%{
load('newexample.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0');
%}

lidPlanes = 1;  
closeThreshold = 0.5; 
farThreshold = 7.0;
lidRansac = 1;
edgeDetect = 1;
edgeThreshold = 0.5;%0.73; %hex
camRansac = 2;
showFigures = true;

[imgs, points] = readFiles(imgDir, imgNames, scan3dDir, scan3dNames);
pointsAll = points;
tic;
%points = deletePlanes(points, lidPlanes, closeThreshold, farThreshold, showFigures);
[rotEst, transEst, S0CamEst, S0LidEst, rEst, sphInliers] = sphereCalib(imgs, points, fu, fv, u0, v0,...
    lidRansac, [], edgeDetect, edgeThreshold, camRansac, showFigures);
toc;
save('examples/HajderExample/S0res.mat', 'S0LidEst', 'S0CamEst', 'sphInliers');
transEst2 = mean(S0LidEst)-mean((rotEst*S0CamEst')'); %#ok<UDIM>
for i = 1:length(imgNames)    
    fileNameBP = strcat('examples/HajderExample/projections/sphereCalibBackproj', num2str(i)); 
    fileNameCP = strcat('examples/HajderExample/projections/sphereCalibScan', num2str(i)); 
    plotBackProjection(imgs{i}, sphInliers{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameBP);
    plotColoredProjection(imgs{i}, pointsAll{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameCP);
end
end

%{
im1=imread("img1.jpg");
im2=imread("img2.jpg");
im3=imread("img3.jpg");
im4=imread("img4.jpg");

imgs=cell(4,1);

imgs{1}=im1;
imgs{2}=im2;
imgs{3}=im3;
imgs{4}=im4;

points1=load("pts1.xyz","-ascii");
points2=load("pts2.xyz","-ascii");
points3=load("pts3.xyz","-ascii");
points4=load("pts4.xyz","-ascii");


points=cell(4,1);

points{1}=points1;
points{2}=points2;
points{3}=points3;
points{4}=points4;


[imgs, points] = readFiles(imgDir, imgNames, scan3dDir, scan3dNames);

fu=1000.0;
fv=1000.0;
u0=960;
v0=600;

lidPlanes = 4;  
closeThreshold = 0; 
lidRansac = 1;
edgeDetect = 1;
edgeThreshold = 0.3;
camRansac = 2;
showFigures = true;

[rot, trans, S0Cam, S0Lid, rCam, sphInliers] = sphereCalib(imgs, points, fu, fv, u0, v0, lidRansac, [] ,edgeDetect, edgeThreshold, camRansac, showFigures);
%}
