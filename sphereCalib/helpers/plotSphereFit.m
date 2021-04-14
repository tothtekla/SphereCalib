function plotSphereFit(points, inliers)
%
% Plot fitted sphere in 3D data
%
figure;
hold on;
axis equal;
scatter3(points(:,1), points(:,2), points(:,3));
scatter3(inliers(:,1), inliers(:,2), inliers(:,3));
legend('points', 'inliers');
end
