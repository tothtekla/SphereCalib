classdef PointSet2D
    % 2D point set
    
    properties
        XY (:,2) double % B by 2 point set
    end
    properties (Dependent)
        N (1,1) double % length of XY
    end
    
    methods
        function N = get.N(obj)
            N = size(obj.XY,1);
        end
        function obj = set.N(obj, val)
            if val < obj.N
                obj.XY = obj.XY(1:val, :);
            elseif val > obj.N
                obj.XY = [obj.XY; zeros(val-obj.N,2)]; 
            end
        end
        
        function out = PointSet2D(obj)
            % Constructor
            if nargin == 0
                return; 
            end
            if isa(obj,'PointSet2D')
                out.XY = obj.XY;
            elseif ismatrix(obj) && ~isempty(obj) && size(obj,2) == 2
                out.XY = obj;
            end
        end
        
        function parametricEllipse = fitEllipse(p, idxs)
            %
            % Estimate ellipse parameters with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % p.XY: 2D ellipse points, N by 2 matrix
            % parametricEllipse: ellipse parameters in order [a, b, ex, ey, theta]
            %
            arguments
                p (1,1) PointSet2D
                idxs = 1:p.N;
             end
            if p.N < 3 
                warning('Dimension N must be greater or equal than 3, no ellipse fitted.');
                parametricEllipse = ParametricEllipse();
                return; 
            end
            if ~isvector(idxs)
                warning('Indices must be a vector, no ellipse fitted.');
            end
            if length(idxs) < 3
                warning('Dimension idxs must be greater or equal than 3, no ellipse fitted.');
                parametricEllipse = ParametricEllipse();
                return; 
            end
            idxN = length(idxs);
            % Add third coordinate z=1 and normalize points
            XY_h = [p.XY(idxs(:), :) ones(idxN, 1)];
            XY_h = sqrt(ones./(sum((XY_h.*XY_h),2)))*ones(1,3).*XY_h;

            % Find the axis drirection and angle of the cone
            wPerCosAlpha = XY_h \ ones(idxN, 1);
            cosAlpha = 1 / norm(wPerCosAlpha);
            w = cosAlpha * wPerCosAlpha;

            % Rotation around the orthogonal vector o with alpha
            sinAlpha = sqrt(1 - cosAlpha^2);
            versinAlpha = 1 - cosAlpha;
            if isequal(w, [0 0 1])
                R = [cosAlpha,   0,    sinAlpha;...
                     0,           1,    0;...
                     -sinAlpha,  0,    cosAlpha]; 
            else
                o = [-w(2) w(1)];
                o = o / norm(o);
                R = [o(1)^2*versinAlpha + cosAlpha, o(1)*o(2)*versinAlpha,         o(2)*sinAlpha;...
                     o(1)*o(2)*versinAlpha,         o(2)^2*versinAlpha + cosAlpha, -o(1)*sinAlpha;...
                     -o(2)*sinAlpha,                o(1)*sinAlpha,                  cosAlpha];
            end
            % Vector from the origin to the direction of the 
            % major ellipse axis ending points
            q_a1 = (R * w);
            q_a2 = (R' * w);
            % Ending points of the major ellipse axes in the image
            a1 = [q_a1(1)/q_a1(3) q_a1(2)/q_a1(3)];
            a2 = [q_a2(1)/q_a2(3) q_a2(2)/q_a2(3)];

            % Ellipse center
            E0 = 0.5*(a1 + a2);
            ex = E0(1); ey = E0(2);
            % Length of the semi-major and semi-minor axis
            a = norm(a2 - a1)/2;
            omega = ex^2 + ey^2 + 1 - a^2;
            b =sqrt((-omega + sqrt(omega^2 + 4*a^2))/2);
            % Rotation angle of the ellipse
            if ey == 0
                theta = 0;
            elseif ex == 0
                theta = 0.5 * pi;
            else
                theta = atan(ey/ex);
            end  

            parametricEllipse = ParametricEllipse([a, b, ex, ey, theta]);
        end
        
        function sphere = fitSphere(p, r)
            %
            % Estimate sphere center with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % XY: 2D ellipse points, N by 2 matrix
            % r: sphere radius
            % s0: sphere center
            %
            arguments
                p (1,1) PointSet2D
                r (1,1) double
            end
            if p.N < 3 
                warning('Dimension N must be greater or equal than 3, no sphere fitted.');
                sphere = Sphere();
                return; 
            end
            % Add third coordinate z=1 and normalize points
            XY_h = [p.XY ones(p.N, 1)];
            XY_h = sqrt(ones./(sum((XY_h.*XY_h),2)))*ones(1,3).*XY_h;

            % Find the axis drirection and angle of the cone
            wPerCosAlpha = XY_h \ ones(p.N, 1);
            cosAlpha = 1 / norm(wPerCosAlpha);
            w = cosAlpha * wPerCosAlpha;

            % Direction vec of the cone axis
            d = r / sqrt(1-cosAlpha^2);

            s0 = d * w;
            
            sphere = Sphere([s0' r]);
        end
        
        %%{
        function [pointSet, sphere] = detectSphere(pointSet, pointSetMin, r, iterations, ransacThreshold, aMin, aMax)
            %
            % Detect sphere projection (ellipse) contour points in 2D pointset using RANSAC
            % Initial inlier set: 3 random points
            % After then fitting sphere with the inliers
            %
            % points : 2D pointset, N by 2 matrix
            % r: sphere radius
            % iterations : no of RANSAC iterations
            % ransacThreshold: Euclidean distance error threshold of RANSAC
            % aMin, aMax: minimum and maximum value of longer ellipse semi-axis for speed-up
            % inliers: best fitted ellipse inlier points
            % S0 : best fitted sphere center
            %
            arguments
                pointSet (:,1) PointSet2D
                pointSetMin (:,1) PointSet2D
                r (1,1) double = 0.25
                iterations (1,1) double = 1000
                ransacThreshold (1,1) double = 5
                aMin (1,1) double = 0.01
                aMax (1,1) double = 1000
            end
            sphere = Sphere.empty(length(pointSet), 0);
            inlierIdxs = [];
            ellipseParamBest = ParametricEllipse();
            f = waitbar(0,'Detecting sphere');
            for i = 1: length(pointSet)
                waitText = strcat({'Detecting '}, num2str(i), '-th sphere...');
                waitbar(i/length(pointSet), f, waitText{1});
                if pointSetMin(i).N < 3
                    %skip
                else
                    for j = 1 : iterations
                        %if mod(j,iterations/20) == 0
                        %    waitbar(j/iterations, f, waitText{1});
                        %end
                        % generate 3 random point 
                        inlierIdxsTmp = randperm(pointSetMin(i).N, 3);
                        % approximate sphere projection parameters with 3 points  
                        ellipseParam = fitEllipse(pointSetMin(i), inlierIdxsTmp);
                        % optional speed-up: longer ellipse semi-axis smaller than aMin or greater than aMax is assumed unrealistic
                        if ellipseParam.a > aMin && ellipseParam.a < aMax
                            % label points
                            %inlierIdxsTmp = classifyEllipsePoints(pointSet.XY, ellipseParam, ransacThreshold);
                            % distance of the points from the ellipse with numerical approximation
                            distances = pointEllipseDistance(pointSetMin(i).XY(:,1), pointSetMin(i).XY(:,2), ellipseParam.a, ...
                            ellipseParam.b, ellipseParam.e0(1), ellipseParam.e0(2), ellipseParam.theta);
                            % if the point fits, save the index
                            inlierIdxs = find(distances < ransacThreshold);
                            % save the best model
                            if(length(inlierIdxsTmp) > length(inlierIdxs))
                                inlierIdxs = inlierIdxsTmp;
                                ellipseParamBest = ellipseParam;
                            end
                        end
                    end
                    distances = pointEllipseDistance(pointSet(i).XY(:,1), pointSet(i).XY(:,2), ellipseParamBest.a, ...
                    ellipseParamBest.b, ellipseParamBest.e0(1), ellipseParamBest.e0(2), ellipseParamBest.theta);
                    % if the point fits, save the index
                    inlierIdxs = find(distances < ransacThreshold);
                    % re-fit a sphere
                    pointSet(i).XY = pointSet(i).XY(inlierIdxs, :);
                    sphere(i) = fitSphere(pointSet(i), r);
                end
            end
            waitbar(1,f,'Sphere detection finished!');
            close(f);
        end
        %}
    end
    
    methods (Static)       
        function dist = pointEllipseDistance(px, py, a, b, ex, ey, theta)
            %
            % Point-ellipse distance with numerical approximation
            % Based on: https://www.iquilezles.org/www/articles/ellipsoids/ellipsoids.htm
            %
            % [px, py] : 2D points
            % [a, b, ex, ey, theta] : ellipse parameters
            % dist : estimated distance
            %
            ab = [a b];
            px0 = cos(theta)*(px - ex) + sin(theta)*(py - ey);
            py0 = - sin(theta)*(px - ex)  + cos(theta)*(py - ey);
            p = [abs(px0)  abs(py0)];
            k0 = p./ab;
            k0 = sqrt(sum(k0.*k0, 2));
            k1 = p./(ab.*ab);
            k1 = sqrt(sum(k1.*k1, 2));
            dist = abs(k0.*(k0-1)./k1); 
        end
    end
end

