function  ellipseParam = fitEllipse3p(XY)
%
% Estimate ellipse parameters with only at least 3 points, if the ellipse 
% is a sphere projection in the image
%
% XY: 2D ellipse points, N by 2 matrix
% ellipseParam: ellipse parameters in order [a, b, ex, ey, theta]
%

% Add third coordinate z=1 and normalize points
XY_h = [XY ones(length(XY), 1)];
XY_h = sqrt(ones./(sum((XY_h.*XY_h),2)))*ones(1,3).*XY_h;

% Find the axis drirection and angle of the cone
wPerCosAlpha = XY_h \ ones(length(XY), 1);
cosAlpha = 1 / norm(wPerCosAlpha);
w = cosAlpha * wPerCosAlpha;

% Rotation around the orthogonal vector o with alpha
sinAlpha = sqrt(1 - cosAlpha^2);
versinAlpha = 1 - cosAlpha;
if isequal(w, [0 0 1])
    R = [cosAlpha,   0,    sinAlpha;...
         0,           1,    0;...
         -sinAlpha,  0,    cosAlpha]; 
else
    o = [-w(2) w(1)];
    o = o / norm(o);
    R = [o(1)^2*versinAlpha + cosAlpha, o(1)*o(2)*versinAlpha,         o(2)*sinAlpha;...
         o(1)*o(2)*versinAlpha,         o(2)^2*versinAlpha + cosAlpha, -o(1)*sinAlpha;...
         -o(2)*sinAlpha,                o(1)*sinAlpha,                  cosAlpha];
end
% Vector from the origin to the direction of the 
% major ellipse axis ending points
q_a1 = (R * w);
q_a2 = (R' * w);
% Ending points of the major ellipse axes in the image
a1 = [q_a1(1)/q_a1(3) q_a1(2)/q_a1(3)];
a2 = [q_a2(1)/q_a2(3) q_a2(2)/q_a2(3)];

% Ellipse center
E0 = 0.5*(a1 + a2);
ex = E0(1); ey = E0(2);
% Length of the semi-major and semi-minor axis
a = norm(a2 - a1)/2;
omega = ex^2 + ey^2 + 1 - a^2;
b =sqrt((-omega + sqrt(omega^2 + 4*a^2))/2);
% Rotation angle of the ellipse
if ey == 0
    theta = 0;
elseif ex == 0
    theta = 0.5 * pi;
else
    theta = atan(ey/ex);
end  

ellipseParam = [a, b, ex, ey, theta];
end