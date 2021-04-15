function [A, B, C, D, E, F] = implSphere2implEllipse(x0, y0, z0, r)
%
% Conversion from implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
%            to projected implicit ellipse     
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%
% Based on Tekla Tóth, Zoltán Pusztai, Levente Hajder: Automatic LiDAR-Camera 
% Calibration of Extrinsic Parameters Using a Spherical Target., ICRA 2020
%
A = y0^2+z0^2-r^2;
B = -2*x0*y0;
C = x0^2+z0^2-r^2;
D = -2*x0*z0;
E = -2*y0*z0;
F = x0^2+y0^2-r*r;
end