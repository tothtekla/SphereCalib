function points_m = gray2meter(edgeDetect, img, edgeThreshold, fu, fv, u0, v0)
%
% Convert gray-scaled image into edge pointset in meters

% get edge points
switch edgeDetect
    case 1
        edgeImg = edge(img,'canny', edgeThreshold);
    case 2
        edgeImg = edge(img,'sobel', edgeThreshold);
end
points_pix = getIdxList(edgeImg);
% swap x and y to have x as horizontal and y as verical axis
points_pix = [points_pix(:, 2) (points_pix(:, 1))];
% normalize coordinates
points_m = pixel2meter(points_pix, fu, fv, u0, v0);