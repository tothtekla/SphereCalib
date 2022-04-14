function [rotEst, transEst, rEst] = helloHajder
close all;
%
% Calibration example on real data
% The radius of the calibration sphere was around 0.3 meter
%
%{
load('hexample.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0');
%}
%{
load('newexample.mat', 'imgDir', 'imgNames', 'scan3dDir', 'scan3dNames',...
    'fu', 'fv', 'u0', 'v0');
%}
%{
fu=fu/2;
fv=fv/2;
u0=u0/2;
v0=v0/2;
%}

%[1250,0,960;0,1250,600;0,0,1]
fu = 625;
u0 = 480;
fv = 625;
v0 = 300;

%{
numFiles = 76;
points = points(1:5:numFiles);
imgs = imgs(1:5:numFiles);
imgNames = imgNames(1:5:numFiles);
scan3dNames = scan3dNames(1:5:numFiles);
%}


idx0 = [52:57, 72:76];%[38:75]%[14:20,37:76];
idx1 = [70:77,34:40, 53:58, 81:85];%[16:58,67:86];
idx2 = [34:40,82:88];%[19:42,74:106];

ptCloudCamSeq = cell(3,1);
ptCloudLidSeq = cell(3,1);
ptCloudCamProjSeq = cell(3,1);

seq = cell(3,1);
seq{1} = idx0;
seq{2} = idx1;
seq{3} = idx2;
seq_rot=cell(3,1);
seq_tran=cell(3,1);
for s = 2%1:3%%!!
numFiles = length(seq{s});
imgNames = cell(numFiles, 1);
scan3dNames = cell(numFiles, 1);
for i = 1:numFiles
    j=seq{s}(i);
    imgNames{i}=strcat('Dev',num2str(s-1),'_Image_w960_h600_fn',num2str(j),'.jpg');
    scan3dNames{i}=strcat('test_fn',num2str(j),'.xyz');
end


%{
fmin = %%%%22%47;
fmax = %%%%%166%137;%61;
imgNames = cell(fmax-fmin+1, 1);
scan3dNames = cell(fmax-fmin+1, 1);
for i = fmin : fmax
    j=i-fmin+1;
    imgNames{j}=strcat('Dev1_Image_w960_h600_fn',num2str(i),'.jpg');
    scan3dNames{j}=strcat('test_fn',num2str(i),'.xyz');
end
%}

lidPlanes = 1;  
closeThreshold = 0.5; 
farThreshold = 3.0;
lidRansac = 1;
edgeDetect = 1;
edgeThreshold = 0.6;%8;%0.5;%0.73; %hex
camRansac = 2;
showFigures = true;%false;% 

imgDir='C:\Users\totht\Downloads\OneDrive_1_1-25-2022\parkolo_gomb_cal_4FPS_pictures\';
%'C:\Users\totht\Downloads\garazs_gomb_cal_4FPS_pictures\';
scan3dDir='C:\Users\totht\Downloads\OneDrive_1_1-25-2022\parkolo_gomb_cal_4FPS_XYZ\';
%'C:\Users\totht\Downloads\garazs_gomb_cal_4FPS_XYZ\';
[imgs, points] = readFiles(imgDir, imgNames, scan3dDir, scan3dNames);

%sumImage = imgs{1}; % Inialize to first image.
avg=zeros(size(imgs{1}));
avg=cast(avg,'uint8');
t=10;
for i = 1:600
    for j=1:960
avg(i,j,1) = mean2([imgs{1}(i,j,1),imgs{2}(i,j,1),imgs{3}(i,j,1),imgs{4}(i,j,1),imgs{4}(i,j,1),imgs{6}(i,j,1)]);
avg(i,j,2) = mean2([imgs{1}(i,j,2),imgs{2}(i,j,2),imgs{3}(i,j,2),imgs{4}(i,j,2),imgs{4}(i,j,2),imgs{6}(i,j,2)]);
avg(i,j,3) = mean2([imgs{1}(i,j,3),imgs{2}(i,j,3),imgs{3}(i,j,3),imgs{4}(i,j,3),imgs{4}(i,j,3),imgs{6}(i,j,3)]);
if abs(avg(i,j,1)-imgs{1}(i,j,1))<t && abs(avg(i,j,2)-imgs{1}(i,j,2))<t && abs(avg(i,j,3)-imgs{1}(i,j,3))<t &&...
    abs(avg(i,j,1)-imgs{2}(i,j,1))<t && abs(avg(i,j,2)-imgs{2}(i,j,2))<t && abs(avg(i,j,3)-imgs{2}(i,j,3))<t &&...
    abs(avg(i,j,1)-imgs{3}(i,j,1))<t && abs(avg(i,j,2)-imgs{3}(i,j,2))<t && abs(avg(i,j,3)-imgs{3}(i,j,3))<t &&...
    abs(avg(i,j,1)-imgs{4}(i,j,1))<t && abs(avg(i,j,2)-imgs{4}(i,j,2))<t && abs(avg(i,j,3)-imgs{4}(i,j,3))<t &&...
    abs(avg(i,j,1)-imgs{5}(i,j,1))<t && abs(avg(i,j,2)-imgs{5}(i,j,2))<t && abs(avg(i,j,3)-imgs{5}(i,j,3))<t &&...
    abs(avg(i,j,1)-imgs{6}(i,j,1))<t && abs(avg(i,j,2)-imgs{6}(i,j,2))<t && abs(avg(i,j,3)-imgs{6}(i,j,3))<t 
    avg(i,j,1)=255;
    avg(i,j,2)=255;
    avg(i,j,3)=255;
 
end
    end
end

imshow(avg);
%for i=2:6 % Read in remaining images.
%  sumImageR = sumImage + imgs{i};
%end
%meanImage = sumImage / 6;
%c=cast(sumImage,'uint8');
%numFiles = 76;
%points = points(1:5:numFiles);
%imgs = imgs(1:5:numFiles);
pointsAll = points;
tic;
%points = deletePlanes(points, lidPlanes, closeThreshold, farThreshold, showFigures);
for i = 1:numFiles
    % detele very close points
    distances = sqrt(sum(points{i}.*points{i}, 2));
    toDelete = find(distances < closeThreshold | distances > farThreshold);
    points{i}(toDelete, :) = [];
end
[rotEst, transEst, S0CamEst, S0LidEst, rEst, sphInliers] = sphereCalib(imgs, points, fu, fv, u0, v0,...
    lidRansac, [], edgeDetect, edgeThreshold, camRansac, showFigures);
toc;
save('S0resJan.mat', 'S0LidEst', 'S0CamEst', 'sphInliers');
transEst2 = mean(S0LidEst)-mean((rotEst*S0CamEst')'); %#ok<UDIM>

ptCloudCam = pointCloud(S0CamEst);
ptCloudLid = pointCloud(S0LidEst);

pcshowpair(ptCloudCam,ptCloudLid,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds before registration')
legend({'Cam','Lid'},'TextColor','w')

tform = pcregistercpd(ptCloudCam,ptCloudLid, 'Transform', 'Rigid');
ptCloudCamProj = pctransform(ptCloudCam,tform);

ptCloudCamSeq{s} = ptCloudCam;
ptCloudLidSeq{s} = ptCloudLid;
ptCloudCamProjSeq{s} = ptCloudCamProj;

pcshowpair(ptCloudCamProj, ptCloudLid,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after registration')
legend({'Cam','Lid'},'TextColor','w')

rotEst = tform.Rotation';
transEst2 = tform.Translation;
seq_rot{s}=rotEst;
seq_trans{s}=transEst2;
end
save('reg_cloud.mat', 'ptCloudCamSeq', 'ptCloudCamProjSeq', 'ptCloudLidSeq')


figure;
hold on;
axis equal;
plotLidar(eye(3), zeros(1,3), 0.5);
for s = 1:3
plotCamera(seq_rot{s}, seq_trans{s}, fu, fv, u0, v0, 0.5);
end
hold off;


colors = [0, 71, 0;
    0, 0, 122;
    112, 0, 112;
    77, 77, 77];

colors = [0, 200, 0;
    0, 0, 200;
    200, 0, 200;
    77, 77, 77];

for i = 1:3
pointscolor=uint8(zeros(ptCloudCamProjSeq{i}.Count,3));
pointscolor(:,1)=colors(i, 1);
pointscolor(:,2)=colors(i, 2);
pointscolor(:,3)=colors(i, 3);
ptCloudCamProjSeq{i}.Color=pointscolor;

pointscolor=uint8(zeros(ptCloudLidSeq{i}.Count,3));
pointscolor(:,1)=colors(4, 1);
pointscolor(:,2)=colors(4, 2);
pointscolor(:,3)=colors(4, 3);
ptCloudLidSeq{i}.Color=pointscolor;
end 



ms = 1500;
figure
hold on;
axis equal;
set(gca,'FontSize',30);
pcshow(ptCloudCamProjSeq{1},'MarkerSize',ms);
pcshow(ptCloudCamProjSeq{2},'MarkerSize',ms);
pcshow(ptCloudCamProjSeq{3},'MarkerSize',ms);
pcshow(ptCloudLidSeq{1},'MarkerSize',ms);
pcshow(ptCloudLidSeq{2},'MarkerSize',ms);
pcshow(ptCloudLidSeq{3},'MarkerSize',ms);
title('Point clouds after registration');
legend({'Camera1','Camera2', 'Camera3', 'LiDAR'});
set(gcf,'color','w');
set(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])


%{
for i = 1:length(imgNames)    
    fileNameBP = strcat('projections/sphereCalibBackproj', num2str(i)); 
    fileNameCP = strcat('projections/sphereCalibScan', num2str(i)); 
    plotBackProjection(imgs{i}, sphInliers{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameBP);
    plotColoredProjection(imgs{i}, pointsAll{i}, rotEst, transEst2, fu, fv, u0, v0, fileNameCP);
end
%}
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
