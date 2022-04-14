function sph_center_reg_evaluation
load('reg_cloud.mat', 'ptCloudCamSeq', 'ptCloudCamProjSeq', 'ptCloudLidSeq');
diffPts = cell(3,1);
avg = [0 0 0];
stdp = [0 0 0];
for i = 1:3
    diffPts{i} = vecnorm(ptCloudLidSeq{i}.Location' - ptCloudCamProjSeq{i}.Location');
    avg(i) = mean(diffPts{i});
    stdp(i) = std(diffPts{i});
end
diffPtsC = vecnorm([ptCloudLidSeq{1}.Location' - ptCloudCamProjSeq{1}.Location',...
                       ptCloudLidSeq{2}.Location' - ptCloudCamProjSeq{2}.Location'...
                       ptCloudLidSeq{3}.Location' - ptCloudCamProjSeq{3}.Location']);
avgC = mean(diffPtsC);
stdC = std(diffPtsC);
end