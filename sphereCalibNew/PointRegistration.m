classdef PointRegistration
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        pts1 PointSet3D
        pts2 PointSet3D
        rotation (3,3) double
        translation (1,3) double
    end

    methods
        function out = PointRegistration(obj1, obj2)
            if nargin == 0
                return; 
            end
            if isa(obj1, 'SphereConverter') && isa(obj2, 'SphereConverter') && ~isempty(obj1)  && ~isempty(obj2) 
                if(size(obj1,2) ~= size(obj2,2))
                    error('Registered pointsets must have the same length.');
                end
                N = size(obj1,2);
                pos1 = zeros(N, 3);
                pos2 = zeros(N, 3);
                toDelete = false(N,1);
                for i = 1 : N
                    if(obj1(i).r ~= 0 && obj2(i).r ~= 0)
                        pos1(i,:) = obj1(i).s0;
                        pos2(i,:) = obj2(i).s0;
                    else
                        toDelete(i) = 1;
                    end
                end
                pos1(toDelete, :) = [];
                pos2(toDelete, :) = [];
                %S0Lid  = S0Lid (~isoutlier(rLid),:);
                %S0Cam  = S0Cam (~isoutlier(rLid),:);
                out.pts1 = PointSet3D(pos1);
                out.pts2 = PointSet3D(pos2);
                [out.rotation, out.translation] = PointRegistration.pointReg(pos1, pos2);
            else
                error('Invalid constructor arguments provided.');
            end
        end
        function plotPointClouds(pointReg)
            arguments
                pointReg (1,1) PointRegistration
            end
            ptCloudCam = pointCloud(pointReg.pts1.XYZ);
            ptCloudLid = pointCloud(pointReg.pts2.XYZ);
            
            colors = [0, 200, 0;
                0, 0, 200;
                200, 0, 200;
                77, 77, 77;
                200, 0, 0;
                200, 200, 0];

            pointscolor=uint8(zeros(ptCloudCam.Count,3));
            pointscolor(:,1)=colors(1, 1);
            pointscolor(:,2)=colors(1, 2);
            pointscolor(:,3)=colors(1, 3);
            ptCloudCam.Color=pointscolor;
            
            pointscolor=uint8(zeros(ptCloudLid.Count,3));
            pointscolor(:,1)=colors(4, 1);
            pointscolor(:,2)=colors(4, 2);
            pointscolor(:,3)=colors(4, 3);
            ptCloudLid.Color=pointscolor;
            
            ms = 500;

            figure;
            hold on;
            axis equal;
            set(gca,'FontSize',30);
            %pcshowpair(ptCloudCam,ptCloudLid,'MarkerSize',50);         
            pcshow(ptCloudCam,'MarkerSize',ms);
            pcshow(ptCloudLid,'MarkerSize',ms);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Point clouds before registration');
            legend({'Cam','Lid'},'TextColor','w');

            tform = pcregistercpd(ptCloudCam,ptCloudLid, 'Transform', 'Rigid');
            ptCloudCamProj = pctransform(ptCloudCam,tform);
            
            pointscolor=uint8(zeros(ptCloudCamProj.Count,3));
            pointscolor(:,1)=colors(1, 1);
            pointscolor(:,2)=colors(1, 2);
            pointscolor(:,3)=colors(1, 3);
            ptCloudCamProj.Color=pointscolor;

            figure;
            hold on;
            axis equal;
            set(gca,'FontSize',30);
            pcshow(ptCloudCamProj,'MarkerSize',ms);
            pcshow(ptCloudLid,'MarkerSize',ms);
            xlabel('X');
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Point clouds after registration with point PPR');
            legend({'Camera','LiDAR'});
            set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
        end
    end

    methods(Static)

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
            %err = rot*pts1 + trans -pts2;
            err = zeros(length(pts1), 1);    
            for i = 1:length(pts1)
                err(i, :) = norm(pts1(i, :)*rot' - pts2(i, :));
            end
            disp('PPR RMSE')
            disp(sqrt(mean((err).^2)));
        end

        function [rot, trans, inlierIdxs] = robustPointRegByRANSAC(pts1, pts2)
            %
            % Robust pointset registration with RANSAC
            % The method filters the outliers of the pointsets and finds an accurate
            % registration based on Arun et al., "Least-squares fitting of two
            % 3-D pointsets."
            % Note: applicable between any N-dimensional pointsets
            %
            % pts1:  first pointset, M by N matrix, M >= N
            % pts2:  second pointset containing the transformed points of the first one
            %        in the same order, M by N matrix
            % rot:   rotation, N by N matrix 
            % trans: translation, N element vector vector
            % idxs: the indices of the points utilized in the registration
            %
            % The mapping :     pts2 = rot * pts1 + trans
            % (no noise assumed)
            %     
            
            iterations = 100000;
            ransacThresholdReproj = 0.2;
            minNoPts = 10;
            numPts = length(pts1);
            inlierIdxs = [];
            
            for i = 1:iterations
                % generate 3 random point 
                minInlierIdxsTmp =  randperm(numPts, minNoPts);
                [rot, trans] = PointRegistration.pointReg(pts1(minInlierIdxsTmp, :),pts2(minInlierIdxsTmp, :));
                % reproject pst1 with the trans. param.s
                pst1Reproj = (rot * pts1')' + trans;
                % error between the reprojected pst1 and pst2
                errorReproj = abs(vecnorm((pst1Reproj - pts2)')');
                % label points
                inlierIdxsTmp = find(errorReproj < ransacThresholdReproj);
                % save the best model
                if(length(inlierIdxsTmp) > length(inlierIdxs))
                    inlierIdxs = inlierIdxsTmp;
                    minInlierIdxs = minInlierIdxsTmp;
                end
            end
            
            [rot, trans] = PointRegistration.pointReg(pts1(minInlierIdxs, :),pts2(minInlierIdxs, :));
            pst1Reproj = (rot * pts1')' + trans;
            errorReproj = abs(vecnorm((pst1Reproj - pts2)')');             
        end


        function plotSensors(rot, trans, fu, fv, u0, v0)
            figure;
            hold on;
            axis equal;
            PointRegistration.plotLidar(eye(3), zeros(1,3), 0.5, 2,2);
            for s = 1 %:3
                PointRegistration.plotCamera(rot, trans, fu, fv, u0, v0, 0.5, 2,2 , [120, 120, 0]/255); %{s}
            end
            hold off;
        end

        function plotCamera( rot, trans, fu, fv, u0, v0, scale,lwRGB, lwEdge, color)
            %
            % Plot camera frustum and axes
            %
            % Extrinsic calibration parameters
            % - rot -- rotation matrix, 3 by 3
            % - trans -- translation vector 
            % Camera instrinsics
            % - fu, fv -- focal length with pixel scale
            % - u0, v0 -- principal point
            % Other
            % scaling of the plot
            %
            points = [0 0 0;
                      u0/fu, v0/fv, 1;
                      u0/fu, -v0/fv, 1;
                      -u0/fu, -v0/fv, 1;
                      -u0/fu, v0/fv, 1;
                      eye(3)];
                  
            points =(rot * scale * points')' + trans;
            
            P_start = [points(1, :); points(1, :); points(1, :); points(1, :); ...
                       points(2, :); points(3, :); points(4, :); points(5, :)];
            P_end = [points(2, :); points(3, :); points(4, :); points(5, :);...
                     points(3, :); points(4, :); points(5, :); points(2, :)];
              
            %Plot camera frustrum
            for i = 1:8
                plot3([P_start(i,1) P_end(i,1)], [P_start(i,2) P_end(i,2)], [P_start(i,3) P_end(i,3)], 'linewidth',lwEdge, 'Color', color);     
            end
            
            %Plot camea axes
            plot3([trans(1) points(6,1)], [trans(2) points(6,2)], [trans(3) points(6,3)], 'r', 'linewidth',lwRGB);
            plot3([trans(1) points(7,1)], [trans(2) points(7,2)], [trans(3) points(7,3)], 'Color', [0, 204, 0]/255, 'linewidth',lwRGB);
            plot3([trans(1) points(8,1)], [trans(2) points(8,2)], [trans(3) points(8,3)], 'b', 'linewidth',lwRGB);
        end

        function plotLidar(rot, trans, scale,lwRGB, lwEdge)
            %
            % Plot schematic LiDAR as a cylinder and axes
            %
            % Extrinsic calibration parameters
            % - rot -- rotation matrix, 3 by 3
            % - trans -- translation vector 
            % Other
            % - scale -- scaling of the plot
            %
            u = 0:0.05:1;
            u = 2 * pi * u;
            N = length(u);
            
            circle1 = [1.0*cos(u);  1.0*sin(u); -0.34*ones(1, N)];
            circle2 = [1.0*cos(u);  1.0*sin(u);  0.34*ones(1, N)];
            points = eye(3);
                  
            circle1 =(rot * scale * circle1)' + trans;
            circle2 =(rot * scale * circle2)' + trans;
            points =(rot * scale * points)' + trans;
              
            %Plot camera frustrum
            for i = 1 :(N - 1)
                plot3([circle1(i,1) circle1(i+1,1)], [circle1(i,2) circle1(i+1,2)], [circle1(i,3) circle1(i+1,3)], 'k', 'linewidth',lwEdge, 'Color', [77, 77, 77]/255);  
                plot3([circle2(i,1) circle2(i+1,1)], [circle2(i,2) circle2(i+1,2)], [circle2(i,3) circle2(i+1,3)], 'k', 'linewidth',lwEdge, 'Color', [77, 77, 77]/255);  
                if mod(i,2) == 0
                    plot3([circle1(i,1) circle2(i,1)], [circle1(i,2) circle2(i,2)], [circle1(i,3) circle2(i,3)], ':', 'linewidth',lwEdge, 'Color', [154, 229, 229]/255); 
                end
            end
            
            %Plot camea axes
            plot3([trans(1) points(1,1)], [trans(2) points(1,2)], [trans(3) points(1,3)], 'r', 'linewidth',lwRGB);
            plot3([trans(1) points(2,1)], [trans(2) points(2,2)], [trans(3) points(2,3)], 'Color', [0, 204, 0]/255, 'linewidth',lwRGB);
            plot3([trans(1) points(3,1)], [trans(2) points(3,2)], [trans(3) points(3,3)], 'b', 'linewidth',lwRGB);
        end

    end
end