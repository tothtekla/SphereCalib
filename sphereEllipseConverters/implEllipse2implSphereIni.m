function [S0, alpha] = implEllipse2implSphereIni(A, B, C, D, E, F, r)
%
% Initial sphere center estimation
% Conversion from implicit ellipse     
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%            to implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
%
% A,..,F: implicit ellipse parameters
% r: sphere radius
% S0: sphere center 
% alpha: scale 
%
const = [   0,      0,      0,   -1/2;
            1,     -1,      0,   -1/2;
            1,      0,     -1,   -1/2;
        1/r^2, -1/r^2, -1/r^2, -1/r^2];
PHI = [A;
       C;
       F;
       B*D/E];
X = abs(const * PHI);
S0 = sqrt(X(1:3) /  X(4)); 
S0(1) = S0(1) * sign(-D);
S0(2) = S0(2) * sign(-E);
alpha = X(4);
end