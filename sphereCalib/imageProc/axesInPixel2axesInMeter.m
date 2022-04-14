function [a_dist_m, b_dist_m] = axesInPixel2axesInMeter(a_dist_pix, b_dist_pix, fu, fv, u0, v0, alpha)
%
% Convert a distance in the image from pixels to meters using camera instrinsic parameters
% dist_pix: end point of the line segment
% angle: rotation of the line segment
%
%a_dist_pix = (a_dist_pix - [u0 v0]);
%b_dist_pix = (b_dist_pix - [u0 v0]);
a_pix = [a_dist_pix zeros(length(a_dist_pix), 1)];
b_pix = [zeros(length(b_dist_pix), 1) b_dist_pix];
for i = 1 : length(alpha)
    R = [ cos(alpha(i)), -sin(alpha(i));
      sin(alpha(i)), cos(alpha(i))];
    a_pix(i, 1:2) = (R * a_pix(i, 1:2)')';
    b_pix(i, 1:2) = (R * b_pix(i, 1:2)')';
end

a_m = a_pix ./ [fu fv];
b_m = b_pix ./ [fu fv];
%a_m = pixel2meter(a_pix, fu, fv, u0, v0);
%b_m = pixel2meter(b_pix, fu, fv, u0, v0);
a_dist_m= vecnorm(a_m,2,2);
b_dist_m= vecnorm(b_m,2,2);
end