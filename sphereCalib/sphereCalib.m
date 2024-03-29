function [rot, trans, S0Cam, S0Lid, rCam, sphInliers] = sphereCalib(imgs, points, fu, fv, u0, v0, lidRansac, rMaxLid, edgeDetect, edgeThreshold, camRansac, showFigures)
%
% 3D-2D sensor calibration using spheres
% 
% lidRansac = 1 and camRansac = 1 are the applied algorithms in 
% Tekla Tóth, Zoltán Pusztai, Levente Hajder:
% "Automatic LiDAR-Camera Calibration of Extrinsic Parameters Using a Spherical Target"
%
% INPUT
% Source data
% - imgs -- image frames
% - points -- 3D point cloud
% Camera instrinsics
% - fu, fv -- focal length with pixel scale
% - u0, v0 -- principal point
% Other flags and parameters
% - lidRansac -- RANSAC algorithms to find the sphere inliers in the 3D point clouds
%                   if 1, adjacency based RANSAC 
%                   if 2, distance based RANSAC
%                   if 3, classical RANSAC with random 4 points 
% - rMaxLid (optional) -- a possible maximal radius for the sphere
%                         if empty, 1.0 is asssumed realistic for real-world scenarios
%                         (it can be useful mostly with synthetic test)  
% - edgeDetect -- edge detector method
%                   if 1, Canny
%                   if 2, Sobel
% - edgeThreshold -- opional threshold parameter for the edgeDetect method
%                   if empty, set automatically
% - camRansac -- RANSAC algorithms to find ellipse inliers in the images 
%                   if 1, approximation with a circle
%                   if 2, detection with sphere projection
% - showFigures -- if true, show camera images and 3D point clouds with the detected inliers 
% 
% OUTPUT
% Extrinsic calibration parameters
% - rot -- rotation matrix, 3 by 3
% - trans -- translation vector 
% Sphere estimation results
% - S0Cam -- Estimated sphere centers from 2D data, N by 3 matrix
% - S0Lid -- Estimated sphere centers from +D data, N by 3 matrix
% - rCam -- Estimated sphere radius 
% - sphInliers -- sphere inliers
%
numScan = length(points);
numImgs = length(imgs);

sphInliers = cell(numScan, 1);
S0Lid = zeros(numScan,3);
rLid = zeros(numScan, 1);
S0Cam = zeros(numImgs,3);

iterationsRLid = 100000;
iterationsALid = 10000;
iterationsDLid = 10000;

ransacThresholdLid = 0.05;
adjacencyThresholdLid = 30;
distanceThresholdLid = 0.15;

iterationsRCam = 10000;
ransacThresholdCam = 4/fu; % n/fu means n pixel max error

rMinLid = 0.10;
if isempty(rMaxLid)
    rMaxLid = 1.0;
end

rMinCam = 10/fu;
rMaxCam = u0/fu;

% detect sphere centers from 3D data
for i = 1:numScan
    % detect sphere inliers and parameters
    switch lidRansac
        case 1
            [sphInliers{i}, S0Lid(i,:), rLid(i)] = detectSphereWithAdjacency(points{i}, iterationsALid, adjacencyThresholdLid, ransacThresholdLid, rMinLid, rMaxLid);
        case 2 
            [sphInliers{i}, S0Lid(i,:), rLid(i)] = detectSphereWithDistance(points{i}, iterationsDLid, distanceThresholdLid, ransacThresholdLid, rMinLid, rMaxLid);
        case 3
            [sphInliers{i}, S0Lid(i,:), rLid(i)] = detectSphereRand4p(points{i}, iterationsRLid, ransacThresholdLid, rMinLid, rMaxLid);
    end
    if showFigures
        plotSphereFit(points{i}, sphInliers{i});
    end
end

% save average estimated radius 
rCam = mean(rLid);

% detect sphere centers from 2D data
for i = 1:numImgs
    % convert image
    img = rgb2gray(imgs{i});
    % get edge points
    switch edgeDetect
        case 1
            edgeImg = edge(img,'canny', edgeThreshold);
        case 2
            edgeImg = edge(img,'sobel', edgeThreshold);
    end
    points_pix = getIdxList(edgeImg);
    % swap x and y to have x as horizontal and y as verical axis
    points_pix = [points_pix(:, 2) (points_pix(:, 1))];
    % normalize coordinates
    points_m = pixel2meter(points_pix, fu, fv, u0, v0);
    % detect sphere projection inliers and sphere parameters
    switch camRansac
        case 1
            [inliers, ellipseImpl] = detectEllipseWithCircle(points_m, iterationsRCam, ransacThresholdCam, rMinCam, rMaxCam);
            [S0CamIni, alphaIni] = implEllipse2implSphereIni(ellipseImpl(1), ellipseImpl(2),...
                                                ellipseImpl(3), ellipseImpl(4),...
                                                ellipseImpl(5), ellipseImpl(6), rCam);
            [S0Cam(i,:), ~] = implEllipse2implSphereOpt(ellipseImpl, rCam, S0CamIni, alphaIni);
        case 2
            [inliers, S0Cam(i,:)] = detectSphereRand3p(points_m, rCam, iterationsRCam, ransacThresholdCam, rMinCam, rMaxCam);
    end
    if showFigures
        plotEllipseFit(points_m, inliers, fu, fv, u0, v0);
    end
end

% pointset registration between sphere centers
[rot, trans] = pointReg(S0Cam,S0Lid);
end


