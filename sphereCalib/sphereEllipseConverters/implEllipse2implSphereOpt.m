function [S0, alpha] = implEllipse2implSphereOpt(ellipseImpl, r, S0, alpha)
%
% Optimal sphere center estimation with Levenberg-Marquard minimization
% Conversion from implicit ellipse     
%              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
%            to implicit sphere             
%              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
%
% ellipseImpl: [A,...,F] implicit ellipse parameters
% r: sphere radius
% S0: sphere center (input: initial, output: optimal)
% alpha : scale (input: initial, output: optimal)
%
const = [0 1 1  0  0  0 -r^2;
         0 0 0 -2  0  0  0;
         1 0 1  0  0  0 -r^2;
         0 0 0  0 -2  0  0;
         0 0 0  0  0 -2  0;
         1 1 0  0  0  0  -r^2];
X_Var =  sym('X_Var', [1 4]);
FUN = @(X_Var) ellipseImpl'-(const *[X_Var(4)*X_Var(1)^2
                            X_Var(4)*X_Var(2)^2
                            X_Var(4)*X_Var(3)^2
                            X_Var(4)*X_Var(1)*X_Var(2)
                            X_Var(4)*X_Var(1)*X_Var(3)
                            X_Var(4)*X_Var(2)*X_Var(3)
                            X_Var(4)]); 
X0 = [S0; alpha];
LB = [-5 -5 0 0];
UB = [ 5  5 25 10000];
options = optimoptions(@lsqnonlin,...
                        'Algorithm',                'levenberg-marquardt',...
                        'Display',                  'iter',...
                        'FunctionTolerance',        1e-20,...
                        'MaxFunctionEvaluations',   3000,...
                        'OptimalityTolerance',      1e-12,...
                        'StepTolerance',            1e-25 );
X = lsqnonlin(FUN, X0, LB, UB, options);
S0 = X(1:3); 
alpha = X(4);
end