function [S0, alpha] = sphereFromEllipseIni(A, B, C, D, E, F, r)
%
% Initial sphere center estimation
%
% A,..,F: implicitellipse parameters
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