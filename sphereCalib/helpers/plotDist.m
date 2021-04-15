function plotDist(ellipseParam, points)
%
% Plot point-ellipse distance function in [-0.5 0.5] interval and some
% points to decide whether they fit or not
%
idxI = -0.5:0.005:0.5;
idxJ = -0.5:0.005:0.5;
N = length(idxI) * length(idxJ);
x = zeros(1,N);
y = zeros(1,N);
d = zeros(1,N);
for i = 1 : N
        x(i) = -0.5 + 0.005 * (floor(i / length(idxI)));
        y(i) = -0.5 + 0.005 * (mod(i, length(idxI)));
        d(i) = pointEllipseDistance(x(i), y(i), ellipseParam(1), ...
        ellipseParam(2), ellipseParam(3), ellipseParam(4), ellipseParam(5));
end

figure;
hold on;
axis equal;
scatter(x, y, [], d);
scatter(points(:,1), points(:,2), 'x');
legend('d', 'pts');
hold off;

end
        