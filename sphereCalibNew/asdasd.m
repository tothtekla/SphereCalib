close all;
%% Read files
%%{
obj1 = 'C:\Users\totht\ELTE\Data\20220311\5_pic';           % imgDir 
obj2 = 'C:\Users\totht\ELTE\Data\20220311\5_cartesians';    % cloudDir
f = FileReaderWriter(obj1, obj2);
[imgs, clouds] = readFiles(f, [6, 70:85]); %106  % 6 20 56:76]);
%% Generate edge points
%%{
fu = 625;
u0 = 480;
fv = 625;
v0 = 300;
imgData = ImageData(imgs, f.imgNames, [fu fv], [u0 v0]);
%}
%%{
[pointSet, pointSetMin] = generateEdges(imgData, 1);

rMinCam = 10/imgData.intrinsic.FocalLength(1);
rMaxCam = imgData.intrinsic.PrincipalPoint(1)/(2*imgData.intrinsic.FocalLength(1));
ransacThresholdCam = 4/imgData.intrinsic.FocalLength(1);
%}

%% Detect Spheres in Images
%%{
[pointSetInliersImg, s0Img] = detectSphere(pointSet,pointSetMin, 0.25, 1000, 1/fu);
%}
%% Detect Spheres in Point Cloud
%{
cloudData = PointCloudData(clouds, f.cloudNames, 16);
pointSet3d = filterCloud(cloudData, closeThreshold, farThreshold);
%}
closeThreshold = 0.5; 
farThreshold = 3.0;
pointSet3d = PointSet3D(clouds, closeThreshold, farThreshold);
[pointSetInliersCld, s0Cld] = detectSphereWithAdjacency(pointSet3d);
%% Pose estimation
pointReg = PointRegistration(s0Img, s0Cld);
plotPointClouds(pointReg);
%}
%%
%{
for i = 1:length(pointSetInliers)
    inliers = true(imgData.intrinsic.ImageSize);
    for j = 1:length(pointSetInliers(i).XY)
        inliers(pointSetInliers(i).XY(j, 2), pointSetInliers(i).XY(j, 1)) = false;
    end
    figure;
    imshow(inliers);
end
%}

%{
%input no. of sensors 1, 3

imgNames=dir('C:\Users\totht\Downloads\OneDrive_1_1-25-2022\ELTE-logo_vissza_4FPS_pictures');
cloudNames=dir('C:\Users\totht\Downloads\OneDrive_1_1-25-2022\ELTE-logo_vissza_4FPS_XYZ');
cloudNameStart = 'test_fn';
cloudNameEnd = '.xyz';


cloudNameMinLength = length(cloudNameStart) + length(cloudNameEnd);

cloudN = size(cloudNames, 1);
cloudIdxs = zeros(cloudN, 1);

for i = 1 : cloudN
   if length(cloudNames(i).name) > cloudNameMinLength &&...
           all(cloudNames(i).name(1:length(cloudNameStart)) == cloudNameStart) &&...
           all(cloudNames(i).name(end-length(cloudNameEnd)+1 : end) == cloudNameEnd)
       if cloudNames(i).bytes == 0
           warningMessage = strcat({'Empty file: '}, cloudNames(i).name);
           warning(warningMessage{1});
       end
       cloudIdxs(i) = str2double(cloudNames(i).name(length(cloudNameStart)+1 : end-length(cloudNameEnd)));
   else
       warningMessage = strcat({'No file loaded: '}, cloudNames(i).name);
       warning(warningMessage{1});
   end
end

cloudIdxs2 = sort(nonzeros(cloudIdxs));
%cloud = 'test_fn*.xyz';
%img = 'Dev*_Image_w960_h600_fn*';
%}


function showRegCloud

load('reg_cloud.mat', 'ptCloudCamSeq', 'ptCloudCamProjSeq', 'ptCloudLidSeq')

alpha = 0.5; % Centripetal 
CamProjSeqCR = cell(3);
ptsC = cell(3);
LidProjSeqCR = cell(3);
ptsL = cell(3);
ptsCProj=cell(3);

for i = 2
    idx = 1;
    CamProjSeqCR{i} = [];
    n = 5;%length(ptCloudCamSeq{i}.Location);
    for j = 9:9+n-3
        distVec = ptCloudCamSeq{i}.Location(j+2, :) - ptCloudCamSeq{i}.Location(j+1, :);
        dist = sqrt(sum(distVec.*distVec));
        step = 1 / (dist*100);
        for t = 0:step:1 %0.01 -> step? 
            pCR = CatmullRomSpline(ptCloudCamSeq{i}.Location(j, :),...
                                 ptCloudCamSeq{i}.Location(j+1, :),...
                                 ptCloudCamSeq{i}.Location(j+2, :),...
                                 ptCloudCamSeq{i}.Location(j+3, :),...
                                 t, alpha);
            CamProjSeqCR{i}(idx, 1:3) = pCR;
            idx = idx + 1;
        end
    end
    ptsC{i} = pointCloud(CamProjSeqCR{i});
end

for i = 2
    idx = 1;
    LidProjSeqCR{i} = [];
    n = 5;%length(ptCloudLidSeq{i}.Location);
    for j = 9:9+n-3
        distVec = ptCloudLidSeq{i}.Location(j+2, :) - ptCloudLidSeq{i}.Location(j+1, :);
        dist = sqrt(sum(distVec.*distVec));
        step = 1 / (dist*100);
        for t = 0:step:1 %0.01 -> step? 
            pCR = CatmullRomSpline(ptCloudLidSeq{i}.Location(j, :),...
                                 ptCloudLidSeq{i}.Location(j+1, :),...
                                 ptCloudLidSeq{i}.Location(j+2, :),...
                                 ptCloudLidSeq{i}.Location(j+3, :),...
                                 t, alpha);
            LidProjSeqCR{i}(idx, 1:3) = pCR;
            idx = idx + 1;
        end
    end
    ptsL{i} = pointCloud(LidProjSeqCR{i});
end

for i = 2
tform = pcregistercpd(ptsC{i} ,ptsL{i}, 'Transform', 'Rigid');
ptsCProj{i} = pctransform(ptsC{i},tform);
end

colors = [0, 71, 0;
    0, 0, 122;
    112, 0, 112;
    77, 77, 77];

colors = [0, 200, 0;
    0, 0, 200;
    200, 0, 200;
    77, 77, 77];

for i = 2
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

pointscolor=uint8(zeros(ptsC{i}.Count,3));
pointscolor(:,1)=colors(i, 1);
pointscolor(:,2)=colors(i, 2);
pointscolor(:,3)=colors(i, 3);
ptsC{i}.Color=pointscolor;

pointscolor=uint8(zeros(ptsL{i}.Count,3));
pointscolor(:,1)=colors(4, 1);
pointscolor(:,2)=colors(4, 2);
pointscolor(:,3)=colors(4, 3);
ptsL{i}.Color=pointscolor;
end 



ms = 1500;
figure
hold on;
axis equal;
set(gca,'FontSize',30);
%pcshow(ptCloudCamProjSeq{1},'MarkerSize',ms);
pcshow(ptCloudCamProjSeq{2},'MarkerSize',ms);
%pcshow(ptCloudCamProjSeq{3},'MarkerSize',ms);
%pcshow(ptCloudLidSeq{1},'MarkerSize',ms);
pcshow(ptCloudLidSeq{2},'MarkerSize',ms);
%pcshow(ptCloudLidSeq{3},'MarkerSize',ms);
for i = 2
    %pcshow(ptsC{i},ms/2);
    pcshow(ptsL{i},'MarkerSize',ms/6);
    pcshow(ptsCProj{i},'MarkerSize',ms/6);
end
title('Point clouds after registration');
legend({'Camera1','Camera2', 'Camera3', 'LiDAR'});
set(gcf,'color','w');
wset(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])

end

function C = CatmullRomSpline(p0, p1, p2, p3, tCR, alpha)
t0 = 0;
t1 = GetT( t0, alpha, p0, p1 );
t2 = GetT( t1, alpha, p1, p2 );
t3 = GetT( t2, alpha, p2, p3 );
t = (1-tCR)*t1 + tCR*t2;

A1 = ( t1-t )/( t1-t0 )*p0 + ( t-t0 )/( t1-t0 )*p1;
A2 = ( t2-t )/( t2-t1 )*p1 + ( t-t1 )/( t2-t1 )*p2;
A3 = ( t3-t )/( t3-t2 )*p2 + ( t-t2 )/( t3-t2 )*p3;
B1 = ( t2-t )/( t2-t0 )*A1 + ( t-t0 )/( t2-t0 )*A2;
B2 = ( t3-t )/( t3-t1 )*A2 + ( t-t1 )/( t3-t1 )*A3;
C  = ( t2-t )/( t2-t1 )*B1 + ( t-t1 )/( t2-t1 )*B2;
end

function t1 = GetT(t0, alpha, p0, p1)
distVec = p1 - p0;
dist = sqrt(sum(distVec.*distVec));
t1 = dist^alpha  + t0;
end
