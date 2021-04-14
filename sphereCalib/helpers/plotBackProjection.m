function plotBackProjection(img, pointsLid_m, rot, trans, fu, fv, u0, v0)
%
% Backprojected 3D points in the image
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
pointsCam_pix = meter2pixel(pointsCam_m, fu, fv, u0, v0);

figure;
hold on;
axis equal;
image(img);
scatter(pointsCam_pix(:, 1), pointsCam_pix(:, 2), 'red', 'filled');
hold off;
end