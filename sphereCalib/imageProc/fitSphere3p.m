function  S0 = fitSphere3p(XY, r)
%
% Estimate sphere center with only at least 3 points, if the ellipse 
% is a sphere projection in the image
%
% XY: 2D ellipse points, N by 2 matrix
% r: sphere radius
% S0: sphere center
%

% Add third coordinate z=1 and normalize points
XY_h = [XY ones(length(XY), 1)];
XY_h = sqrt(ones./(sum((XY_h.*XY_h),2)))*ones(1,3).*XY_h;

% Find the axis drirection and angle of the cone
wPerCosAlpha = XY_h \ ones(length(XY), 1);
cosAlpha = 1 / norm(wPerCosAlpha);
w = cosAlpha * wPerCosAlpha;

% Direction vec of the cone axis
d = r / sqrt(1-cosAlpha^2);

S0 = d * w;
end