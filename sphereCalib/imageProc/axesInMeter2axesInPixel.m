function [a_dist_pix, b_dist_pix] = axesInMeter2axesInPixel(a_dist_m, b_dist_m, fu, fv, u0, v0, alpha)
%
% Convert a distance in the image from meters to pixels using camera instrinsic parameters
% dist_pix: end point of the line segment
% angle: rotation of the line segment
%
%a_dist_pix = (a_dist_pix - [u0 v0]);
%b_dist_pix = (b_dist_pix - [u0 v0]);
a_m = [a_dist_m zeros(length(a_dist_m), 1)];
b_m = [zeros(length(b_dist_m), 1) b_dist_m];
for i = 1 : length(alpha)
    R = [ cos(alpha(i)), -sin(alpha(i));
      sin(alpha(i)), cos(alpha(i))];
    a_m(i, 1:2) = (R * a_m(i, 1:2)')';
    b_m(i, 1:2) = (R * b_m(i, 1:2)')';
end

a_pix = a_m .* [fu fv];
b_pix = b_m .* [fu fv];
%a_m = pixel2meter(a_pix, fu, fv, u0, v0);
%b_m = pixel2meter(b_pix, fu, fv, u0, v0);
a_dist_pix = vecnorm(a_pix,2,2);
b_dist_pix = vecnorm(b_pix,2,2);
end