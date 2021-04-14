function plotColoredProjection(img, pointsLid_m, rot, trans, fu, fv, u0, v0)
%
% Color 3D points with the image pixels
% The shifts show the calibration error
%
% Source data 
% - img -- image data
% - pointsLid_m -- 3D points to reproject into the image 
% Extrinsic calibration parameters
% - rot -- rotation matrix, 3 by 3
% - trans -- translation vector 
% Camera instrinsics
% - fu, fv -- focal length with pixel scale
% - u0, v0 -- principal point
%
pointsCam_m = (rot' * (pointsLid_m - trans)')';
pointsCam_m = pointsCam_m(:,1:2) ./ pointsCam_m(:,3);
pointsCam_pix = floor(meter2pixel(pointsCam_m, fu, fv, u0, v0));
colorsLid = zeros(length(pointsCam_pix), 3);
img = im2double(img);
for i = 1: length(pointsCam_pix)
    colorsLid(i, :) = img(pointsCam_pix(i,2), pointsCam_pix(i, 1), :);
end

figure;
hold on;
axis equal;
scatter3(pointsLid_m(:, 1), pointsLid_m(:, 2), pointsLid_m(:, 3), [], colorsLid, 'filled');
hold off;
end