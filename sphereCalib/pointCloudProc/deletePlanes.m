function points =  deletePlanes(points, numPlanes, closeThreshold, showFigures)
%
% Delete planes with Sequential RANSAC 
%
% points - 3D point cloud
% numPlanes - no. of planes to delete
% pointsOut - remaining 3D point cloud
% showFigures -- if true, show camera images and 3D point clouds with the detected inliers 
%
numScans = length(points);
iterations = 10000;
ransacThreshold = 0.04;
planes = cell(numScans, numPlanes);
closePoints =  cell(numScans, 1);

for i = 1:numScans
    % detele very close points
    distances = sqrt(sum(points{i}.*points{i}, 2));
    toDelete = find(distances < closeThreshold);
    closePoints{i} = points{i}(toDelete, :);
    points{i}(toDelete, :) = [];
    % dete planes
    for j = 1:numPlanes
        numPts = length(points{i});
        inlierIdxs = [];
        for k = 1 : iterations
            % generate 3 random point
            inlierIdxsTmp = randperm(numPts, 3);
            % find sphere parameters
            [p, n] = fitPlane3p(points{i}(inlierIdxsTmp,:));
            % label points
            inlierIdxsTmp = classifyPlanePoints(points{i}, p, n, ransacThreshold);       
            % save the best model
            if(length(inlierIdxsTmp) > length(inlierIdxs))
                inlierIdxs = inlierIdxsTmp;
            end
        end
        % delete plane
        outlierIdxs = 1:numPts;
        outlierIdxs(inlierIdxs) = [];
        planes{i,j} = points{i}(inlierIdxs, :);
        points{i} = points{i}( outlierIdxs, :);
    end
end

if showFigures
    for i = 1:numScans
        figure;
        hold on;
        axis equal; 
        scatter3(closePoints{i}(:,1), closePoints{i}(:,2), closePoints{i}(:,3));
        for j = 1:numPlanes
            scatter3(planes{i,j}(:,1), planes{i,j}(:,2), planes{i,j}(:,3));
        end
        scatter3(points{i}(:,1), points{i}(:,2), points{i}(:,3));
    end
end

end