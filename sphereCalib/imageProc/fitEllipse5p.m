function ellipseImpl = fitEllipse5p(points)
%
% Ellipse fitting algorithm at least with 5 points
% points: 2D ellipse contour pointset, n by 2 matrix
% ellipseImpl: A,...,F implicit ellipse parameters
%
% Based on:
% https://www.mathworks.com/matlabcentral/fileexchange/22684-ellipse-fit-direct-method
% 
% Modifications:
% - A2, A4 and A5 are not divided by 2
% - all coeff-s multiplied by -1 if A would be negative
% 
% Original comments:
%
%  Direct ellipse fit, proposed in article
%    A. W. Fitzgibbon, M. Pilu, R. B. Fisher
%     "Direct Least Squares Fitting of Ellipses"
%     IEEE Trans. PAMI, Vol. 21, pages 476-480 (1999)
%
%  Our code is based on a numerically stable version
%  of this fit published by R. Halir and J. Flusser
%
%     Input:  XY(n,2) is the array of coordinates of n points x(i)=XY(i,1), y(i)=XY(i,2)
%
%     Output: A = [a b c d e f]' is the vector of algebraic 
%             parameters of the fitting ellipse:
%             ax^2 + bxy + cy^2 +dx + ey + f = 0
%             the vector A is normed, so that ||A||=1
%
%  This is a fast non-iterative ellipse fit.
%
%  It returns ellipses only, even if points are
%  better approximated by a hyperbola.
%  It is somewhat biased toward smaller ellipses.
%
%
centroid = mean(points);   % the centroid of the data set
D1 = [(points(:,1)-centroid(1)).^2, (points(:,1)-centroid(1)).*(points(:,2)-centroid(2)),...
      (points(:,2)-centroid(2)).^2];
D2 = [points(:,1)-centroid(1), points(:,2)-centroid(2), ones(size(points,1),1)];
S1 = D1'*D1;
S2 = D1'*D2;
S3 = D2'*D2;
T = -inv(S3)*S2';
if ( sum(sum(isinf(T))) || sum(sum(isnan(T))))
    msg = 'Error: Matrix T is invalid';
    warning(msg);
    ellipseImpl = ones(1, 6);
    return;
end
M = S1 + S2*T;
M = [M(3,:)./2; -M(2,:); M(1,:)./2];
[evec,~] = eig(M);
cond = 4*evec(1,:).*evec(3,:)-evec(2,:).^2;
A1 = evec(:,cond>0);
A = [A1; T*A1];
if(isempty(A))
    msg = 'Error: Matrix A is invalid';
    warning(msg);
    ellipseImpl = ones(1,6);
    return;
end
A4 = A(4) - 2*A(1)*centroid(1) - A(2)*centroid(2);
A5 = A(5) - 2*A(3)*centroid(2)  -A(2)*centroid(1);
A6 = A(6) + A(1)*centroid(1)^2 + A(3)*centroid(2)^2 +...
     A(2)*centroid(1)*centroid(2) - A(4)*centroid(1) - A(5)*centroid(2);
A(4) = A4;  
A(5) = A5;  
A(6) = A6;
ellipseImpl = sign(A(1))*(A/norm(A))';
end