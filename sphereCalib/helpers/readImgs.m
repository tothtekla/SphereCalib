function [imgs] = readImgs(imgDir, imgNames)
%
% Read images
%
% 
% INPUT
% Source file infos
% - imgDir, imgNames -- image path and file names
%
% OUTPUT
% Data in cell arrays
% - imags -- images 
%
numImgs = length(imgNames);
if numImgs < 3
    error('Not enough input image file names (second argument).');
end

imgs = cell(numImgs, 1);

for i= 1:numImgs
    imgFile = strcat(imgDir,imgNames(i));
    imgs{i} = imread(char(imgFile));
    %imgs{i} = rgb2gray(imgs{i});
    %imgs{i} = imnoise(imgs{i},'gaussian',0,0.01);
    %{
    f = figure;
    hold on;
    for s = 0.01:0.01:0.19
        J = imnoise(imgs{i},'gaussian',0,s);
        imshow(J);
        pause(3);
    end
    %}
end

end