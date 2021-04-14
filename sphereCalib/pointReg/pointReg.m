function [rot, trans] = pointReg(pts1, pts2)
%
% Pointset registration 
% based on Arun et al., "Least-squares fitting of two 3-D pointsets."
% Note: applicable between any N-dimensional pointsets
%
% pts1:  first pointset, M by N matrix, M >= N
% pts2:  second pointset containing the transformed points of the first one
%        in the same order, M by N matrix
% rot:   rotation, N by N matrix 
% trans: translation, N element vector vector
%
% The mapping :     pts2 = rot * pts1 + trans
%
offset1 = mean(pts1);
offset2 = mean(pts2);

pts1 = pts1 - offset1;
pts2 = pts2 - offset2;

H = zeros(3);
for i=1:length(pts2)
	H = H + (pts1(i, :)'*(pts2(i, :)));
end

[U , ~, V] = svd(H);
rot = V*U';
trans = offset2 - offset1;
end
