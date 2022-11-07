classdef PointSet3D
    % 2D point set
    
    properties
        XYZ (:,3) double % B by 3 point set
    end
    properties (Dependent)
        N (1,1) double % length of XYZ
    end
    
    methods
        function N = get.N(obj)
            N = size(obj.XYZ,1);
        end
        function obj = set.N(obj, val)
            if val < obj.N
                obj.XYZ = obj.XYZ(1:val, :); 
            elseif val > obj.N
                obj.XYZ = [obj.XYZ; zeros(val-obj.N,3)]; 
            end
        end
        
        function out = PointSet3D(obj, closeThreshold, farThreshold)
            arguments
                obj
                closeThreshold (1,1) double = 0
                farThreshold (1,1) double = Inf
            end
            % Constructor
            if nargin == 0
                return; 
            end
            if isa(obj,'PointSet3D')
                out.XYZ = obj.XYZ;
            elseif ismatrix(obj) && ~isempty(obj) && size(obj,2) == 3
                % detele very close and far points
                distances = sqrt(sum(obj.*obj, 2));
                toDelete = distances < closeThreshold | distances > farThreshold;
                obj(toDelete, :) = [];
                % save to out
                out.XYZ = obj;
            elseif iscell(obj)  && ~isempty(obj) && size(obj,2) == 1
                out = PointSet3D.empty(size(obj,1), 0);
                for i = 1 : size(obj,1)
                    % detele very close and far points
                    distances = sqrt(sum(obj{i}.*obj{i}, 2));
                    toDelete = distances < closeThreshold | distances > farThreshold;
                    % dev mode : plot in and out points
                    %  PointSet3D.plotFilteredPointSet(obj{i}, toDelete);
                    obj{i}(toDelete, :) = [];
                    % save to out
                    out(i) = PointSet3D(obj{i});
                end
            else
                error('Invalid constructor arguments provided.');
            end
        end
    
        function [pointSet, sphere] = detectSphereWithAdjacency(pointSet, iterations, adjacencyThreshold, ransacThreshold, rMin, rMax)
            % 
            % Detect sphere surface points in 3D point cloud using RANSAC
            % Initial inlier set: 1 random point + k-NN where k = adjacencyThreshold
            %
            % Based on Tekla Tóth, Levente Hajder: "Robust Fitting of Geometric Primitives on LiDAR Data"
            %
            % points : 3D point cloud, N by 3 matrix
            % iterations : no. of RANSAC iterations
            % adjacencyThreshold : select the closest subset of point withtis size
            % ransacThreshold: Euclidean distance error threshold of RANSAC
            % rMin, rMax: minimum and maximum value of sphere radius for speed-up
            % inliers: best fitted sphere inlier points
            % S0: best fitted sphere center
            % r:  best fitted sphere radius
            %
            arguments
                pointSet (:,1) PointSet3D
                iterations (1,1) double = 1000
                adjacencyThreshold (1,1) double = 100 % 0.15
                ransacThreshold (1,1) double = 0.05
                rMin (1,1) double = 0.05
                rMax (1,1) double = 0.5
            end
            sphere = SphereConverter.empty(length(pointSet), 0);
            fig = figure;
            f = waitbar(0,'Detecting sphere');
            for i = 2 : length(pointSet)
                inlierIdxsBest = [];
                waitText = strcat({'Detecting '}, num2str(i), '-th sphere...');
                waitbar(i/length(pointSet), f, waitText{1});
                if pointSet(i).N < 4
                    warningMessage = strcat({'Few points in the point set: '}, num2str(i));
                    warning(warningMessage{1});
                    %skip
                else
                    %data structures for knn-search
                    MdlKDT = KDTreeSearcher(pointSet(i).XYZ);
                    closestPoint = knnsearch(MdlKDT, [0 0 0], 'K', 1);
                    closestDist = norm(pointSet(i).XYZ(closestPoint, :)) + rMin;
                    for j = 1 : iterations
                        % generate 1 random point
                        randIdx = randi(pointSet(i).N); 
                        % select initial subset
                        inlierIdxs = knnsearch(MdlKDT, pointSet(i).XYZ(randIdx, :), 'K', adjacencyThreshold);
                        %inlierIdxs = rangesearch(MdlKDT, pointSet(i).XYZ(randIdx, :), adjacencyThreshold);
                        %inlierIdxs = cell2mat(inlierIdxs);
                        if(length(inlierIdxs) >= 4 )
                            % find sphere parameters
                            sphere(i) =  fitSphereNonit(pointSet(i), inlierIdxs);
                            % dev mode: plot results
                            %  plotSphereWithDist(pointSet(i), pointSet(i).XYZ(inlierIdxs, :), sphere(i), [], fig);
                            % optional speed-up: a sphere radius smaller
                            % than rMin or greater than rMax is assumed
                            % unrealistic; moreover, the sphere not contains the origin
                            % and the sphere center is further than the 
                            % closest point in the pointset + rMin
                            if sphere(i).r > rMin && sphere(i).r < rMax && ...
                                    norm(sphere(i).s0) > closestDist
                                    %norm(sphere(i).s0) > sphere(i).r && ...                                    
                                % label points       
                                % distance of the points from the ellipse with numerical approximation
                                distances = pointSphereDistance(pointSet(i), ...
                                            sphere(i).s0, sphere(i).r);
                                occlusion = pointSphereOcclusion(pointSet(i), ...
                                            sphere(i).s0, sphere(i).r);
                                % if the point fits, and visible from the lid pos, save the index
                                inlierIdxs = find(distances < ransacThreshold & ~occlusion);
                                % save the best model
                                if(length(inlierIdxs) > length(inlierIdxsBest))
        	                        inlierIdxsBest = inlierIdxs;
                                    % dev mode: show inliers
                                    % plotSphereWithDist(pointSet(i), pointSet(i).XYZ(inlierIdxsBest, :), sphere(i), ...
                                    %        distances, fig);
                                end
                            end
                        end
                    end
                    % re-fit a sphere
                    pointSetBest = PointSet3D(pointSet(i).XYZ(inlierIdxsBest, :));
                    sphere(i) = fitSphereLsq(pointSetBest);
                    % dev mode: show inliers
                    distances = pointSphereDistance(pointSet(i), ...
                                            sphere(i).s0, sphere(i).r);
                    plotSphereWithDist(pointSet(i), pointSetBest.XYZ, sphere(i), distances, fig);
                    pointSet(i) = pointSetBest;

                end
            end    
            waitbar(1,f,'Sphere detection finished!');
            close(f);
        end

    
        function sphere = fitSphereNonit(p, idxs)
            %
            % Robust fast sphere fitting algorithm 
            % based on Sumith YD, "Fast Geometric Fit Algorithm for Sphere Using Exact Solution"
            %
            % x, y, z: input coordinates
            % S0: estimated sphere center
            % r: estimated radius
            %
            arguments
                p (1,1) PointSet3D
                idxs = 1:p.N;
            end
            if p.N < 4 
                warning('Dimension N must be greater or equal than 4, no sphere fitted.');
                sphere = SphereConverter();
                return; 
            end
            if ~isvector(idxs)
                warning('Indices must be a vector, no sphere fitted.');
                sphere = SphereConverter();
                return; 
            end
            if length(idxs) < 3
                warning('Dimension idxs must be greater or equal than 4, no sphere fitted.');
                sphere = SphereConverter();
                return; 
            end
            
            mx = mean(p.XYZ(idxs(:), 1));
            my = mean(p.XYZ(idxs(:), 2));
            mz = mean(p.XYZ(idxs(:), 3));

            x = p.XYZ(idxs(:), 1) - mx;
            y = p.XYZ(idxs(:), 2) - my;
            z = p.XYZ(idxs(:), 3) - mz;

            Sx = sum(x);
            Sy = sum(y);
            Sz = sum(z);
            
            Sxx = sum(x.*x); Syy = sum(y.*y);
            Szz = sum(z.*z); Sxy = sum(x.*y);
            Sxz = sum(x.*z); Syz = sum(y.*z);
            
            Sxxx = sum(x.*x.*x); 
            Syyy = sum(y.*y.*y);
            Szzz = sum(z.*z.*z); 
            Sxyy = sum(x.*y.*y);
            Sxzz = sum(x.*z.*z);
            Sxxy = sum(x.*x.*y);
            Sxxz = sum(x.*x.*z); 
            Syyz =sum(y.*y.*z);
            Syzz = sum(y.*z.*z);
            
            A1 = Sxx +Syy +Szz;
            
            a = 2*Sx*Sx-2*p.N*Sxx;
            b = 2*Sx*Sy-2*p.N*Sxy;
            c = 2*Sx*Sz-2*p.N*Sxz;
            d = -p.N*(Sxxx +Sxyy +Sxzz)+A1*Sx;
            e = 2*Sx*Sy-2*p.N*Sxy;
            f = 2*Sy*Sy-2*p.N*Syy;
            g = 2*Sy*Sz-2*p.N*Syz;
            h = -p.N*(Sxxy +Syyy +Syzz)+A1*Sy;
            j = 2*Sx*Sz-2*p.N*Sxz;
            k = 2*Sy*Sz-2*p.N*Syz;
            l = 2*Sz*Sz-2*p.N*Szz;
            m = -p.N*(Sxxz +Syyz + Szzz)+A1*Sz;
            
            delta = a*(f*l - g*k)-e*(b*l-c*k) + j*(b*g-c*f);
            x0 = (d*(f*l-g*k) -h*(b*l-c*k) +m*(b*g-c*f))/delta;
            y0 = (a*(h*l-m*g) -e*(d*l-m*c) +j*(d*g-h*c))/delta;
            z0 = (a*(f*m-h*k) -e*(b*m-d*k) +j*(b*h-d*f))/delta;
            r = sqrt(x0^2+y0^2+z0^2+(A1-2*(x0*Sx+y0*Sy+z0*Sz))/p.N);
            sphere = SphereConverter([x0+mx y0+my z0+mz r]);
        end
        
        function sphere = fitSphereLsq(p, idxs)
            %
            % Sphere fitting with an iterative least-squares method 
            % based on David Eberly, "Least Squares Fitting of Data"
            %
            % points: input coordinates, N by 3 matrix
            % S0: estimated sphere center
            % r: estimated sphere radius
            %
            arguments
                p (1,1) PointSet3D
                idxs = 1:p.N;
            end

            sphere = SphereConverter();
            if p.N < 4 
                warning('Dimension N must be greater or equal than 4, no sphere fitted.');
                return; 
            end
            if ~isvector(idxs)
                warning('Indices must be a vector, no sphere fitted.');
                return; 
            end
            if length(idxs) < 3
                warning('Dimension idxs must be greater or equal than 4, no sphere fitted.');
                return; 
            end

            iterations = 1000; % Maximal no. of fixpoint iterations
            meanPts = mean(p.XYZ);
            numPts = length(p.XYZ);
            p.XYZ = p.XYZ - meanPts;
            
            numS0Ini = floor(numPts / 4);
            S0Ini = zeros(numS0Ini, 3);
            idxs = randperm(length(p.XYZ));
            for i = 1 : numS0Ini
                j = (i - 1) * 4  + 1;
                rnd4 = PointSet3D(p.XYZ(idxs(j:(j+3)),:));
                sphere =  fitSphere4p(rnd4);
                S0Ini(i, :) = sphere.s0;
            end
            S0 = median(S0Ini(~isnan(S0Ini)));
            
            for j = 1 : iterations
                r_avg = 0;
                dir_avg = [0 0 0];
                S0_tmp = S0; 
                for i = 1 : numPts
                    dir_i = S0 - p.XYZ(i,1:3);
                    r_i = sqrt(dot(dir_i,dir_i));
                    r_avg = r_avg + r_i;
                    dir_avg = dir_avg + ( (S0 - p.XYZ(i,:)) / r_i);
                end
            
                S0 = (r_avg * dir_avg) / numPts^2; 
                r = r_avg / numPts;
                diff_new = S0 - S0_tmp; 
                if(eq(diff_new,[0 0 0]))
                    break;
                end
            end
            S0 = S0 + meanPts;
            sphere = SphereConverter([S0 r]);
        end

        function sphere = fitSphere4p(p)
            %
            % Minimal sphere fitting algorithm with 4 points
            % Used for initial sphere center estimation
            %
            % p: sphere surface points, 4 by 3 matrix
            % S0: sphere center
            %
            % The center point considered as intersection of planes
            % n and midP are the normal vectors and a points lying on a plane
            % S0 = ni^(-1) * [<n1,midP1> ; <n2,midP2> ; <n3,midP3> ; <n4,midP4>]
            %
            arguments
                p PointSet3D
            end
            midP = zeros(4, 3);
            n = zeros(4, 3);
            d = [0; 0; 0];
            sphere = SphereConverter();

            for i = 1:4
                midP(i,:) = (p.XYZ(i,:) + p.XYZ(mod(i,4)+1,:)) / 2;
                n(i,:) = p.XYZ(i,:) - midP(i,:);
                n(i,:) = n(i,:) / norm(n(i,:));
                d(i) = n(i,:) * midP(i,:)';
            end
            
            if isempty(find(isnan(n), 1)) || isempty(find(isinf(n), 1))
                sphere.s0 = (n \ d)';   
                sphere.r = norm(sphere.s0 - p.XYZ(1, :));
            else
                % skip: return sphere with zero values 
            end
        end

        function distances = pointSphereDistance(pointSet, S0, r)
            arguments
                pointSet (1,1) PointSet3D
                S0 (1,3) double
                r (1,1) double
            end
            % point distance from the sphere center
            radii = S0 - pointSet.XYZ;
            radii = sqrt(sum(radii.*radii,2));
            % radius error
            distances = abs(r - radii);
        end

        function occlusion = pointSphereOcclusion(pointSet, S0, r)
            arguments
                pointSet (1,1) PointSet3D
                S0 (1,3) double
                r (1,1) double
            end
            % point max distance from the sphere center
            t = norm(S0);
            maxDist = sqrt(t*t - r*r);
            d = sqrt((sum((pointSet.XYZ.*pointSet.XYZ),2)));
            % occlusion
            occlusion = d > maxDist; 
        end
        
        function plotSphereWithDist(pointSet, sph, sphere, dist, fig)
            arguments
                pointSet (1,1) PointSet3D
                sph (:,3) double
                sphere SphereConverter = empty.SphereConverter()
                dist (:,1) double = []
                fig (1,1) matlab.ui.Figure = figure
            end
            % TODO: with sphere contour
            cmatrix1 = ones(length(sph), 3).*[1 0 0];
            cmatrix2 = ones(pointSet.N, 3).*[0 0 1];
            if(~isempty(dist))
                c = parula;
                mx = max(dist);
                t = dist / mx;
                for i = 1 : pointSet.N
                    idx = uint8(t(i) * 256);
                    if idx > 256
                        idx = 256;
                    end
                    if idx < 1
                        idx = 1;
                    end
                    cmatrix2(i, :) = c(idx, :);
                end
            end
            clf(fig);
            hold on;
            axis equal;
            pcPointSet = pointCloud(pointSet.XYZ, 'Color', cmatrix2);
            pcSphere = pointCloud(sph, 'Color', cmatrix1);
            pcshow(pcPointSet, 'MarkerSize', 1000);
            pcshow(pcSphere, 'MarkerSize', 1000);
            if(~isempty(sphere))
                sphereFit = generateSpherePoints(sphere);
                cmatrix3 = ones(length(sphereFit), 3).*[0 1 0];
                pcshow(pointCloud(sphereFit, 'Color', cmatrix3));
            end
            legend('Points', 'Sphere', 'Fitted');
        end
    end
    
    methods(Static)   
        function plotFilteredPointSet(pointSet, toDelete)
            arguments
                pointSet (:, 3) double
                toDelete (:, 1) logical
            end
            cmatrix1 = ones(sum(toDelete), 3).*[1 0 0];
            cmatrix2 = ones(sum(~toDelete), 3).*[0 0 1];
            figure;
            axis equal;
            hold on;
            ptsIn = pointCloud(pointSet(toDelete, :), 'Color',cmatrix1);
            ptsOut = pointCloud(pointSet(~toDelete, :), 'Color',cmatrix2);
            pcshow(ptsIn);
            pcshow(ptsOut);
            legend('In', 'Out');
            hold off;
        end     
    end
end

