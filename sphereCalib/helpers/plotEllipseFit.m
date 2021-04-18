function plotEllipseFit(points, inliers, fu, fv, u0, v0)
%
% Plot fitted ellipse in 2D data
%
figure;
hold on;
grid on;
axis equal;
set(gca, 'YDir','reverse')
xlim([-u0/fu u0/fu]);
ylim([-v0/fv v0/fv]);
scatter(points(:, 1), points(:, 2), 'x');
scatter(inliers(:, 1), inliers(:, 2), 'x');
legend('edges', 'inliers');
hold off;
end