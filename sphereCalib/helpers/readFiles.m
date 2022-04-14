function [imgs, points] = readFiles(imgDir, imgNames, scan3dDir, scan3dNames)
%
% Read image and point cloud data
%
% 
% INPUT
% Source file infos
% - imgDir, imgNames -- image path and file names
% - scan3dDir, scan3dNames -- poit cloud path and file names
%
% OUTPUT
% Data in cell arrays
% - imags -- images 
% - points -  3D point cloud
%
numImgs = length(imgNames);
numScan = length(scan3dNames);
if numImgs < 3
    error('Not enough input image file names (second argument).');
end

if numScan < 3 
    error('Not enough input 3D scan file names (fourth argument).');
end

if numImgs ~= numScan
    error('Number of input images and 3D scans are different (second and fourth argument).');
end

imgs = cell(numImgs, 1);
points = cell(numScan, 1);

%%{
for i= 1:numScan
    scan3dFile = strcat(scan3dDir,scan3dNames(i));
   
    imgFile = strcat(imgDir,imgNames(i));
    if ~isfile(scan3dFile)
        warning(strcat('Warning: file does not exist: ', num2str(i)));
        %i=i-1;
        %numScan=numScan-1;
    else if ~isfile(imgFile)
            warning(strcat('Warning: file does not exist: ', num2str(i)));
            %i=i-1;
            %numScan=numScan-1;
        else
            points{i} = importdata(char(scan3dFile));
            points{i} = points{i}(:, 1:3);
            imgs{i} = imread(char(imgFile));
        end
    end
end
%}

points=points(~cellfun(@isempty,points));
imgs=imgs(~cellfun(@isempty,imgs));

%{
for i= 1:numImgs
end
%}

end