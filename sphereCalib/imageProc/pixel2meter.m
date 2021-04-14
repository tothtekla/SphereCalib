function points_m = pixel2meter(points_pix, fu, fv, u0, v0)
%
% Convert image pixel to meters using camera instrinsic parameters
%
points_m = (points_pix - [u0 v0]) ./ [fu fv];
end