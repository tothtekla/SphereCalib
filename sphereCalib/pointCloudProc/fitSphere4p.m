function [S0, r] = fitSphere4p(points)
%
% Minimal sphere fitting algorithm with 4 points
% Used for initial sphere center estimation
%
% points: sphere surface points, 4 by 3 matrix
% S0: sphere center
%
% The center point considered as intersection of planes
% n and midP are the normal vectors and a points lying on a plane
% S0 = ni^(-1) * [<n1,midP1> ; <n2,midP2> ; <n3,midP3> ; <n4,midP4>]
%
midP = zeros(4, 3);
n = zeros(4, 3);
d = [0; 0; 0];

for i = 1:4
    midP(i,:) = (points(i,:) + points(mod(i,4)+1,:)) / 2;
    n(i,:) = points(i,:) - midP(i,:);
    n(i,:) = n(i,:) / norm(n(i,:));
    d(i) = n(i,:) * midP(i,:)';
end

if isempty(find(isnan(n), 1)) || isempty(find(isinf(n), 1))
    S0 = (n \ d)';   
    r = norm(S0 - points(1, :));
else
    S0 = [0; 0; 0];  
    r = 0;
end
end

