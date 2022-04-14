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

