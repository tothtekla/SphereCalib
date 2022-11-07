function showCR
load('test5_dev0.mat', 's0Img', 's0Cld');
res = 10000; %100;
N = length(s0Img);
pos1 = zeros(N, 3);
pos2 = zeros(N, 3);
toDelete = false(N,1);
for i = 1 : N
    if(s0Img(i).r ~= 0 && s0Cld(i).r ~= 0)
        pos1(i,:) = s0Img(i).s0;
        pos2(i,:) = s0Cld(i).s0;
    else
        toDelete(i) = 1;
    end
end
pos1(toDelete, :) = [];
pos2(toDelete, :) = [];
s0Img = pos1;
s0Cld = pos2;


ptCloudCamProjSeq = pointCloud(s0Img);
ptCloudLidSeq = pointCloud(s0Cld);


diffPts = cell(3,1);
avg = [0 0 0];
stdp = [0 0 0];
diffPts = vecnorm(ptCloudLidSeq.Location' - ptCloudCamProjSeq.Location');
disp('Original avg mean');
avg = mean(diffPts)
stdp = std(diffPts)

colors = [0, 71, 0;
    0, 0, 122;
    112, 0, 112;
    77, 77, 77];

colors = [0, 200, 0;
    0, 0, 200;
    200, 0, 200;
    77, 77, 77];


    
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

ms = 500;
figure
hold on;
axis equal;
set(gca,'FontSize',30);
%pcshow(ptCloudCamProjSeq{1},'MarkerSize',ms);
pcshow(ptCloudCamProjSeq,'MarkerSize',ms);
%pcshow(ptCloudCamProjSeq{3},'MarkerSize',ms);
%pcshow(ptCloudLidSeq{1},'MarkerSize',ms);
pcshow(ptCloudLidSeq,'MarkerSize',ms);
%pcshow(ptCloudLidSeq{3},'MarkerSize',ms);

title('Point clouds before registration');
legend({'Camera1','LiDAR'});
set(gcf,'color','w');
%wset(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])



%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% normal 
[tform, ~, merr] = pcregistercpd(ptCloudCamProjSeq ,ptCloudLidSeq, 'Transform', 'Rigid');
ptCloudCamProjSeq2 = pctransform(ptCloudCamProjSeq,tform);

diffPts = cell(3,1);
avg = [0 0 0];
stdp = [0 0 0];
diffPts = vecnorm(ptCloudLidSeq.Location' - ptCloudCamProjSeq2.Location');
disp('Registered avg mean err');
avg = mean(diffPts)
stdp = std(diffPts)
merr

pointscolor=uint8(zeros(ptCloudCamProjSeq2.Count,3));
pointscolor(:,1)=colors(1, 1);
pointscolor(:,2)=colors(1, 2);
pointscolor(:,3)=colors(1, 3);
ptCloudCamProjSeq2.Color=pointscolor;

figure
hold on;
axis equal;
set(gca,'FontSize',30);
%pcshow(ptCloudCamProjSeq{1},'MarkerSize',ms);
pcshow(ptCloudCamProjSeq2,'MarkerSize',ms);
%pcshow(ptCloudCamProjSeq{3},'MarkerSize',ms);
%pcshow(ptCloudLidSeq{1},'MarkerSize',ms);
pcshow(ptCloudLidSeq,'MarkerSize',ms);
%pcshow(ptCloudLidSeq{3},'MarkerSize',ms);

title('Point clouds after registration');
legend({'Camera1','LiDAR'});
set(gcf,'color','w');
%wset(gca,'color','w');
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%CRS

%{
diffPtsC = vecnorm([ptCloudLidSeq{1}.Location' - ptCloudCamProjSeq{1}.Location',...
                       ptCloudLidSeq{2}.Location' - ptCloudCamProjSeq{2}.Location'...
                       ptCloudLidSeq{3}.Location' - ptCloudCamProjSeq{3}.Location']);
avgC = mean(diffPtsC);
stdC = std(diffPtsC);
%}


alpha = 0.5; % Centripetal 
CamProjSeqCR = [];
ptsC = [];
LidProjSeqCR = [];
LidProjSeqKB = [];
ptsL = [];
ptsCProj = [];
    
idx = 1;
n = length(s0Cld);
for j = 1:n-3
    distVec = s0Cld(j+2, :) - s0Cld(j+1, :);
    dist = sqrt(sum(distVec.*distVec));
    step = 1 / (dist*res);
    for t = 0:step:1 %0.01 -> step? 
        pCR = CatmullRomSpline(s0Cld(j, :),...
                             s0Cld(j+1, :),...
                             s0Cld(j+2, :),...
                             s0Cld(j+3, :),...
                             t, alpha);
        pKB = KochanekBartelsSpline(s0Cld(j, :),...
                             s0Cld(j+1, :),...
                             s0Cld(j+2, :),...
                             s0Cld(j+3, :),...
                             t);
        LidProjSeqCR(idx, 1:3) = pCR;
        LidProjSeqKB(idx, 1:3) = pKB;
        idx = idx + 1;
    end
end
ptsL = pointCloud(LidProjSeqCR);
ptsL2 = pointCloud(LidProjSeqKB);


idx = 1;
CamProjSeqCR = [];
CamProjSeqKB = [];
n = length(s0Img);
for j = 1:n-3
    distVec = s0Img(j+2, :) - s0Img(j+1, :);
    dist = sqrt(sum(distVec.*distVec));
    step = 1 / (dist*res);
    for t = 0:step:1 %0.01 -> step? 
        pCR = CatmullRomSpline(s0Img(j, :),...
                             s0Img(j+1, :),...
                             s0Img(j+2, :),...
                             s0Img(j+3, :),...
                             t, alpha);
        pKB = KochanekBartelsSpline(s0Img(j, :),...
                             s0Img(j+1, :),...
                             s0Img(j+2, :),...
                             s0Img(j+3, :),...
                             t);
        CamProjSeqCR(idx, 1:3) = pCR;
        CamProjSeqKB(idx, 1:3) = pKB;
        idx = idx + 1;
    end
end
ptsC = pointCloud(CamProjSeqCR);
ptsC2 = pointCloud(CamProjSeqKB);

[tform, ~, merr] = pcregistercpd(ptsC ,ptsL, 'Transform', 'Rigid');
ptsCProj = pctransform(ptsC,tform);
ptsC2Proj = pctransform(ptsC2,tform);
ptCloudCamProjSeq = pctransform(ptCloudCamProjSeq,tform);

diffPts = cell(3,1);
avg = [0 0 0];
stdp = [0 0 0];
diffPts = vecnorm(ptCloudLidSeq.Location' - ptCloudCamProjSeq.Location');
disp('CRS avg mean');
avg = mean(diffPts)
stdp = std(diffPts)
merr

colors = [0, 71, 0;
    0, 0, 122;
    112, 0, 112;
    77, 77, 77];

colors = [0, 200, 0;
    0, 0, 200;
    200, 0, 200;
    77, 77, 77];


    
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

pointscolor=uint8(zeros(ptsL.Count,3));
pointscolor(:,1)=colors(4, 1);
pointscolor(:,2)=colors(4, 2);
pointscolor(:,3)=colors(4, 3);
ptsL.Color=pointscolor;

pointscolor=uint8(zeros(ptsL2.Count,3));
pointscolor(:,1)=colors(3, 1);
pointscolor(:,2)=colors(3, 2);
pointscolor(:,3)=colors(3, 3);
ptsL2.Color=pointscolor;


ms = 1500;
figure
hold on;
axis equal;
set(gca,'FontSize',30);
%pcshow(ptCloudCamProjSeq{1},'MarkerSize',ms);
pcshow(ptCloudCamProjSeq,'MarkerSize',ms);
%pcshow(ptCloudCamProjSeq{3},'MarkerSize',ms);
%pcshow(ptCloudLidSeq{1},'MarkerSize',ms);
pcshow(ptCloudLidSeq,'MarkerSize',ms);
%pcshow(ptCloudLidSeq{3},'MarkerSize',ms);

    %pcshow(ptsC{i},ms/2);
    pcshow(ptsL,'MarkerSize',ms/6);
    pcshow(ptsL2,'MarkerSize',ms/6);
    pcshow(ptsCProj,'MarkerSize',ms/6);
    pcshow(ptsC2Proj,'MarkerSize',ms/6);

title('Point clouds after registration');
legend({'Camera1','LiDAR'});
set(gcf,'color','w');
%wset(gca,'color','w');
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

function C = KochanekBartelsSpline(p0, p1, p2, p3, tCR)

t = tCR;

T = 0; 
B = 0; 
C = 0;

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
