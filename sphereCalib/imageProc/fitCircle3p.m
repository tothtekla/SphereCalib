function [C0, r] = fitCircle3p(points)
%
% Minimal circle fitting algorithm with 3 points
%
% points: circle edge coordinates, 3 by 2 matrix
% C0: estimated circle center
% r: estimated radius
%
% The center point considered as intersection of lines
% n and midP are the normal vectors and a points lying on a line
% C0 = n^(-1) * [<n1,midP1> ; <n2,midP2>]
%
midP = zeros(2, 2);
n = zeros(2, 2);
d = [0; 0];

for i = 1:2
    midP(i,:) = (points(i,:) + points(i+1,:)) / 2;
    n(i,:) = points(i,:) - midP(i,:);
    n(i,:) = n(i,:) / norm(n(i,:));
    d(i) = n(i,:) * midP(i,:)';
end

if isempty(find(isnan(n), 1)) || isempty(find(isinf(n), 1))
    C0 = (n \ d)';  
    r = norm(C0 - points(1, :));
else
    C0 = [0; 0];  
    r = 0;
end
end