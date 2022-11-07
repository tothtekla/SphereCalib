function showSplines

load('test5_dev0.mat', 's0Img', 's0Cld');
N = length(s0Img);
pos1 = zeros(N, 3);
pos2 = zeros(N, 3);
toDelete = false(N,1);
for i = 1 : N
    if(s0Img(i).r ~= 0 && s0Cld(i).r ~= 0) % Delete invalid sphere estimations
        pos1(i,:) = s0Img(i).s0;
        pos2(i,:) = s0Cld(i).s0;
    else
        toDelete(i) = 1;
    end
end
pos1(toDelete, :) = [];
pos2(toDelete, :) = [];

%% Pose estimation
% pointReg = PointRegistration(s0Img, s0Cld);
% plotPointClouds(pointReg);

s0Img = pos1;   % 3D sphere centers from image
s0Cld = pos2;   % 3D sphere centers from LiDAR point cloud

%% Point sets before registration
ptCloudCamProjSeq = pointCloud(s0Img); % pointCloud of image-based 3D point sequence
ptCloudLidSeq = pointCloud(s0Cld);     % pointCloud of LiDAR-based 3D point sequence


% Mean error and standard deviation between the point sets
diffPts = vecnorm(ptCloudLidSeq.Location' - ptCloudCamProjSeq.Location');
avg = mean(diffPts);
stdp = std(diffPts);
disp('Original mean error and standard deviation ');
disp(avg);
disp(stdp);

% Colorize point clouds
colors = [0, 200, 0;
    0, 0, 200;
    200, 0, 200;
    77, 77, 77;
    200, 0, 0;
    200, 200, 0];
    
pointscolor=uint8(zeros(ptCloudCamProjSeq.Count,3));
pointscolor(:,1)=colors(1, 1);
pointscolor(:,2)=colors(1, 2);
pointscolor(:,3)=colors(1, 3);
ptCloudCamProjSeq.Color=pointscolor;

pointscolor=uint8(zeros(ptCloudLidSeq.Count,3));
pointscolor(:,1)=colors(4, 1);
pointscolor(:,2)=colors(4, 2);
pointscolor(:,3)=colors(4, 3);
ptCloudLidSeq.Color=pointscolor;

% Show point sets before registration
ms = 500;
%{
figure
hold on;
axis equal;
set(gca,'FontSize',20);
pcshow(ptCloudCamProjSeq,'MarkerSize',ms);
pcshow(ptCloudLidSeq,'MarkerSize',ms);
title('Point clouds before registration');
legend({'Camera','LiDAR'});
set(gcf,'color','w');
%wset(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
%}


%% Point sets after registration using Coherent Point Drift algorithm 
[tform, ~, merr] = pcregistercpd(ptCloudCamProjSeq ,ptCloudLidSeq, 'Transform', 'Rigid');
ptCloudCamProjSeq2 = pctransform(ptCloudCamProjSeq,tform); % Transformed image-based point sequence

% Mean error and standard deviation between the point sets after registration
diffPts = vecnorm(ptCloudLidSeq.Location' - ptCloudCamProjSeq2.Location');
avg = mean(diffPts);
stdp = std(diffPts);
disp('CPD Registered mean error and standard deviation ');
disp(avg);
disp(stdp);
disp(merr);

% Colorize new point cloud
pointscolor=uint8(zeros(ptCloudCamProjSeq2.Count,3));
pointscolor(:,1)=colors(1, 1);
pointscolor(:,2)=colors(1, 2);
pointscolor(:,3)=colors(1, 3);
ptCloudCamProjSeq2.Color=pointscolor;

% Show point sets after registration
%{
figure
hold on;
axis equal;
set(gca,'FontSize',20);
pcshow(ptCloudCamProjSeq2,'MarkerSize',ms);
pcshow(ptCloudLidSeq,'MarkerSize',ms);
title('Point clouds after registration with CPD');
legend({'Camera','LiDAR'});
set(gcf,'color','w');
%wset(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
%}

%% Point sets after registration using Coherent Point Drift algorithm 
[tform, ~, merr] = pcregistericp(ptCloudCamProjSeq ,ptCloudLidSeq);
ptCloudCamProjSeq4 = pctransform(ptCloudCamProjSeq,tform); % Transformed image-based point sequence

% Mean error and standard deviation between the point sets after registration
diffPts = vecnorm(ptCloudLidSeq.Location' - ptCloudCamProjSeq4.Location');
avg = mean(diffPts);
stdp = std(diffPts);
disp('ICP Registered mean error and standard deviation ');
disp(avg);
disp(stdp);
disp(merr);

% Colorize new point cloud
pointscolor=uint8(zeros(ptCloudCamProjSeq4.Count,3));
pointscolor(:,1)=colors(1, 1);
pointscolor(:,2)=colors(1, 2);
pointscolor(:,3)=colors(1, 3);
ptCloudCamProjSeq4.Color=pointscolor;

% Show point sets after registration
%{
figure
hold on;
axis equal;
set(gca,'FontSize',20);
pcshow(ptCloudCamProjSeq4,'MarkerSize',ms);
pcshow(ptCloudLidSeq,'MarkerSize',ms);
title('Point clouds after registration with ICP');
legend({'Camera','LiDAR'});
set(gcf,'color','w');
%wset(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
%}

%% Point set expanson using spline interpolation
%%% Linear
%%% Catmull-Rom
%%% Kochanek-Bartels

alpha = 0.5; % Centripetal CR
%res = 200; %100; % resolution -> eg. if res = 100, new point interpolated / cm
%resToTest = [0.025, 0.05 0.1 0.25 0.5 0.75 1.00];
%resToTest = 1./ (resToTest/100);
resToTest = [4000, 2000, 1000,400,200,133,100];
merr = zeros(3, length(resToTest));
tform = cell(3, length(resToTest));
for i = 1 : length(resToTest) 
    ptsL3 = addLIpoints(s0Cld, resToTest(i));
    ptsC3 = addLIpoints(s0Img, resToTest(i));
    [tform{1, i}, ~, merr(1, i)] = pcregistercpd(ptsC3 ,ptsL3, 'Transform', 'Rigid');

    ptsL  = addCRpoints(s0Cld, resToTest(i), alpha);
    ptsC = addCRpoints(s0Img, resToTest(i), alpha);
    [tform{2, i}, ~, merr(2, i)] = pcregistercpd(ptsC ,ptsL, 'Transform', 'Rigid');
    
    ptsL2 = addKBpoints(s0Cld, resToTest(i));
    ptsC2 = addKBpoints(s0Img, resToTest(i));
    [tform{3, i}, ~, merr(3, i)] = pcregistercpd(ptsC2 ,ptsL2, 'Transform', 'Rigid');
end

fu = 1262.6;
fv = 1267.4;
u0 = 934.6;
v0 = 659.5;


%PointRegistration.plotSensors(tform{1,1}.Rotation', tform{1,1}.Translation, fu, fv, u0, v0);
%{
color = [[1, 0, 0];[0, 0, 1]; [0.8, 0.8, 0]];
figure;
hold on;
axis equal;
PointRegistration.plotLidar(eye(3), zeros(1,3), 0.25, 2, 2);
for i = 1:3
    for j = 1:length(resToTest) 
        tr = tform{i, j}.Rotation;
        tt = tform{i, j}.Translation;
        PointRegistration.plotCamera(tr', tt, fu, fv, u0, v0, 0.25, 2,2 , color(i, :) * j / length(resToTest)); %{s}
    end
end
hold off;
%}

color = [[1, 0, 0];[0, 0, 1]; [0.8, 0.8, 0]];
figure;
hold on;
axis equal;
PointRegistration.plotLidar(eye(3), zeros(1,3), 0.25, 2, 2);
for i = 1:3
    for j = 1
        tr = tform{i, j}.Rotation;
        tt = tform{i, j}.Translation;
        PointRegistration.plotCamera(tr', tt, fu, fv, u0, v0, 0.25, 2,2 , color(i, :) * length(resToTest) - 1 / length(resToTest)); %{s}
    end
end
legend('LiDAR', 'Camera_{LI}', 'Camera_{CRS}', 'Camera_{LKBS}');
hold off;

%% Extended point sets after registration using Coherent Point Drift algorithm
% Registration based on CR points
[tform, ~, merr] = pcregistercpd(ptsC ,ptsL, 'Transform', 'Rigid');
% Transform all camera-based point sets (original + extended versions)
ptsCProj = pctransform(ptsC,tform); 
ptsC2Proj = pctransform(ptsC2,tform);
ptsC3Proj = pctransform(ptsC3,tform);
ptCloudCamProjSeq3 = pctransform(ptCloudCamProjSeq,tform);

% Mean error and standard deviation between the point sets after extended registration
diffPts = vecnorm(ptCloudLidSeq.Location' - ptCloudCamProjSeq3.Location');
avg = mean(diffPts);
stdp = std(diffPts);
disp('Extended registered mean error and standard deviation ');
disp(avg);
disp(stdp);

% Colorize point clouds    
pointscolor=uint8(zeros(ptCloudCamProjSeq3.Count,3));
pointscolor(:,1)=colors(1, 1);
pointscolor(:,2)=colors(1, 2);
pointscolor(:,3)=colors(1, 3);
ptCloudCamProjSeq3.Color=pointscolor;

pointscolor=uint8(zeros(ptsC.Count,3));
pointscolor(:,1)=colors(1, 1);
pointscolor(:,2)=colors(1, 2);
pointscolor(:,3)=colors(1, 3);
ptsC.Color=pointscolor;

pointscolor=uint8(zeros(ptsC2Proj.Count,3));
pointscolor(:,1)=colors(2, 1);
pointscolor(:,2)=colors(2, 2);
pointscolor(:,3)=colors(2, 3);
ptsC2Proj.Color=pointscolor;

pointscolor=uint8(zeros(ptsC3Proj.Count,3));
pointscolor(:,1)=colors(6, 1);
pointscolor(:,2)=colors(6, 2);
pointscolor(:,3)=colors(6, 3);
ptsC3Proj.Color=pointscolor;

pointscolor=uint8(zeros(ptsL.Count,3));
pointscolor(:,1)=colors(2, 1);
pointscolor(:,2)=colors(2, 2);
pointscolor(:,3)=colors(2, 3);
ptsL.Color=pointscolor;

pointscolor=uint8(zeros(ptsL2.Count,3));
pointscolor(:,1)=colors(6, 1);
pointscolor(:,2)=colors(6, 2);
pointscolor(:,3)=colors(6, 3);
ptsL2.Color=pointscolor;

pointscolor=uint8(zeros(ptsL3.Count,3));
pointscolor(:,1)=colors(5, 1);
pointscolor(:,2)=colors(5, 2);
pointscolor(:,3)=colors(5, 3);
ptsL3.Color=pointscolor;

ms = 1500;
figure
hold on;
axis equal;
set(gca,'FontSize',30);

%pcshow(ptCloudCamProjSeq3,'MarkerSize',ms);
pcshow(ptCloudLidSeq,'MarkerSize',ms);

pcshow(ptsL3,'MarkerSize',ms/6);
pcshow(ptsL,'MarkerSize',ms/6);
pcshow(ptsL2,'MarkerSize',ms/6);
%pcshow(ptsCProj,'MarkerSize',ms/6);
%pcshow(ptsC2Proj,'MarkerSize',ms/6);
%pcshow(ptsC3Proj,'MarkerSize',ms/6);

%title('Point clouds after registration');
legend('Control points', 'LI', 'CRS', 'KBS');
%legend({'\color{white} Camera','\color{white} LiDAR', '\color{white} L_{CR}',...
%    '\color{white} L_{KB}', '\color{white} L_{LI}','\color{white} C_{CR}',...
%    '\color{white} C_{KB}', '\color{white} C_{LI}'});

set(gcf,'color','w');
%wset(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])

end

function pts = addLIpoints(s0, res)
ProjSeqLI = [];
idx = 1;
n = length(s0);
for j = 1:n-1
    distVec = s0(j+1, :) - s0(j, :);
    dist = sqrt(sum(distVec.*distVec));
    step = 1 / (dist*res);
    for t = 0:step:1
        pLI = LinearInterpolation(s0(j, :),...
                             s0(j+1, :),...
                             t);
        ProjSeqLI(idx, 1:3) = pLI;
        idx = idx + 1;
    end
end
pts = pointCloud(ProjSeqLI);
end

function pts = addCRpoints(s0, res, alpha)
ProjSeqCR = [];
idx = 1;
n = length(s0);
for j = 1:n-3
    distVec = s0(j+2, :) - s0(j+1, :);
    dist = sqrt(sum(distVec.*distVec));
    step = 1 / (dist*res);
    for t = 0:step:1
        pCR = CatmullRomSpline(s0(j, :),...
                             s0(j+1, :),...
                             s0(j+2, :),...
                             s0(j+3, :),...
                             t, alpha);
        ProjSeqCR(idx, 1:3) = pCR;
        idx = idx + 1;
    end
end
pts = pointCloud(ProjSeqCR);
end

function pts = addKBpoints(s0, res)
ProjSeqKB = [];
idx = 1;
n = length(s0);
for j = 1:n-3
    distVec = s0(j+2, :) - s0(j+1, :);
    dist = sqrt(sum(distVec.*distVec));
    step = 1 / (dist*res);
    T = 0; 
    B = 0.5; 
    C = 0.5;
    for t = 0:step:1
        pKB = KochanekBartelsSpline(s0(j, :),...
                             s0(j+1, :),...
                             s0(j+2, :),...
                             s0(j+3, :),...
                             t, T, B, C);
        ProjSeqKB(idx, 1:3) = pKB;
        idx = idx + 1;
    end
end
pts = pointCloud(ProjSeqKB);
end

function C = LinearInterpolation(p0, p1, t)
C = t * p0 + (1-t) * p1;
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

function C = KochanekBartelsSpline(p0, p1, p2, p3, t, T, B, C)

w11 = (1 - T) * (1 + B) * (1 + C) / 2;
w12 = (1 - T) * (1 - B) * (1 - C) / 2; 
w21 = (1 - T) * (1 + B) * (1 - C) / 2;
w22 = (1 - T) * (1 - B) * (1 + C) / 2;

m1 = w11 * (p1 - p0) + w12 * (p2 - p1);
m2 = w21 * (p2 - p1) + w22 * (p3 - p2);

C = ( 2*t^3 - 3*t^2 + 1) * p1 + ...
    (   t^3 - 2*t^2 + t) * m1 + ...
    (-2*t^3 + 3*t^2)     * p2 + ...
    (   t^3 -   t^2)     * m2;
end
