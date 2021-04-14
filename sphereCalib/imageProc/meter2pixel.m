function points_pix = meter2pixel(points_m, fu, fv, u0, v0)
%
% Convert image point in meters to pixel using camera instrinsic parameters
%
points_pix = (points_m .* [fu fv]) + [u0 v0];
end