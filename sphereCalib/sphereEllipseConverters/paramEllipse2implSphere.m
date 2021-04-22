function  [x0, y0, z0] = paramEllipse2implSphere(a, b, ex, ey, theta, r)
%
% Conversion from projected parametric ellipse     
%              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
%                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey  
%            to implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
% 

% Get unit vectors directed to major ellipse axis enging points
T = [a*cos(theta), -b*sin(theta), ex;...
     a*sin(theta),  b*cos(theta), ey;...
     0,               0,          1];
a1 = (T * [-1, 0, 1]')';
a2 = ( T * [1 , 0, 1]')';
q_a1 = a1 / norm(a1);
q_a2 = a2 / norm(a2);

% Angle bisector
w = (q_a2 + q_a1) / 2;
w = w / norm(w);

% Cone angle and direction
cos2alpha = q_a1*q_a2';
d = sqrt(2)*r / sqrt(1-cos2alpha);

S = d * w;
x0 = S(1);
y0 = S(2);
z0 = S(3);
end