function implEllipse = implSphereVec2implEllipseVec(implSphere)
%
% Conversion from implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
%            to projected implicit ellipse     
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%
% Based on Tekla Tóth, Zoltán Pusztai, Levente Hajder: Automatic LiDAR-Camera 
% Calibration of Extrinsic Parameters Using a Spherical Target., ICRA 2020
%
[A, B, C, D, E, F] = implSphere2implEllipse(implSphere(1), implSphere(2), implSphere(3), implSphere(4));
implEllipse = [A, B, C, D, E, F];
end