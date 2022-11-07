function xyz2ply(fileDir, idxs)



idxs = 71:120;
cloudDir =  'C:\Users\totht\ELTE\Data\20220930\2cartesians';
plyDir = 'C:\Users\totht\ELTE\Data\20220930\2ply';

%idxs = 244:262;
%cloudDir =  'C:\Users\totht\ELTE\Data\20220722-sakktábla-traj\ptc';
%plyDir = 'C:\Users\totht\ELTE\Data\20220722-sakktábla-traj\ply_2';

%idxs = 119:150; 
%cloudDir = 'C:\Users\totht\ELTE\Data\20220915-sakktábla-traj-tta\sakktabla_4cam_4fps_cartesians';
%'C:\Users\totht\ELTE\Data\20220722-sakktábla-traj\ptc';
%plyDir = 'C:\Users\totht\ELTE\Data\20220915-sakktábla-traj-tta\sakktabla_4cam_4fps_ply_2';

cloudNameStart = 'test_fn';
cloudNameEnd = '.xyz';

plyNameStart = 'Dev1_Image_w1920_h1200_fn'; %Dev1
plyNameEnd = '.ply';

N = length(idxs);
clouds = cell(N, 1); 
f = waitbar(0,'Converting point clouds');
for i = 1 : N
    waitText = strcat({'Reading point clouds: '}, num2str(i), '/', num2str(N));
    waitbar(i/N, f, waitText{1});
    cloudFile = strcat(cloudDir,'\', cloudNameStart, num2str(idxs(i)), cloudNameEnd);
    if ~isfile(cloudFile)
        errorMessage = strcat({'File does not exist: '}, num2str(idxs(i)));
        error(errorMessage{1});
    end
    clouds{i} = readmatrix(char(cloudFile), 'FileType', 'text'); %importdata(char(cloudFile));
    clouds{i} = clouds{i}(1:2:end, 1:3);  %% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    plyFile = strcat(plyDir,'\', plyNameStart, num2str(idxs(i)), plyNameEnd);
    writePly(plyFile, clouds{i});
end
waitbar(1,f,'Point cloud reading finished!');
clouds=clouds(~cellfun(@isempty,clouds)); 
close(f);
end 

function writePly(fileName, points)
    fileID = fopen(fileName, 'w');
    fprintf(fileID,'ply\n');
    fprintf(fileID,'format ascii 1.0\n');
    fprintf(fileID,'element vertex %u\n', length(points));
    fprintf(fileID,'property float x\n');
    fprintf(fileID,'property float y\n');
    fprintf(fileID,'property float z\n');
    %fprintf(fileID,'property uchar red\n');
    %fprintf(fileID,'property uchar green\n');
    %fprintf(fileID,'property uchar blue\n');
    fprintf(fileID,'end_header\n');
    data = [points]';
    fprintf(fileID,'%3.7f %3.7f %3.7f\n', data); % %3u %3u %3u\n', data);
    fclose(fileID);
end