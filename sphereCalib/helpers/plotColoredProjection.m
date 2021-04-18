function plotColoredProjection(img, pointsLid_m, rot, trans, fu, fv, u0, v0, fileName)
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
% Output info
% - fileName -- if not empty, save colored point cloud
%
pointsCam_m = (rot' * (pointsLid_m - trans)')';
pointsCam_mh = pointsCam_m(:,1:2) ./ pointsCam_m(:,3);
pointsCam_pix = floor(meter2pixel(pointsCam_mh, fu, fv, u0, v0));
colorsLid = zeros(length(pointsCam_pix), 3);
[r, c, ~] = size(img);
for i = 1: length(pointsCam_pix)
    idxI = pointsCam_pix(i,2);
    idxJ = pointsCam_pix(i,1);
    if idxI > 0 && idxI < r+1 && idxJ > 0 && idxJ < c+1 
        colorsLid(i, :) = img(idxI, idxJ, :);
    end
end
colorsLid(pointsCam_m(:,3) < 0, :) = 0;

if ~ isempty(fileName)
    writePly(fileName, pointsLid_m, colorsLid);
end
colorsLid = colorsLid/256;

figure;
hold on;
axis equal;
scatter3(pointsLid_m(:, 1), pointsLid_m(:, 2), pointsLid_m(:, 3), [], colorsLid, 'filled');
plotCamera(rot, trans, fu, fv, u0, v0, 0.5);
plotLidar(eye(3), zeros(1,3), 0.5);
hold off;

end