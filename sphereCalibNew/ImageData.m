classdef ImageData
    
    properties
        imgs (:, 1) cell 
        intrinsic cameraIntrinsics
        imgNames
    end
    
    methods
        % todo: names length eq to imgs
        function out = ImageData(obj, imgNames, focalLength, principalPoint)
            arguments
                obj
                imgNames (:, 1) cell
                focalLength (2,1) double = [1, 1]
                principalPoint (2,1) double = [1, 1]
            end
            narginchk(1, inf);
            if isa(obj, 'ImageData')
                out.imgs = obj.imgs;
                out.intrinsic = obj.intrinsic;
                out.imgNames = obj.imgNames;
            elseif nargin >= 2 && iscell(obj) && all(class(cell2mat(obj)) == 'uint8')
                out.imgs = obj;
                out.imgNames = imgNames(1:length(obj));
                out.intrinsic = cameraIntrinsics(focalLength,...
                                                 principalPoint,...
                                                 size(obj{1}, 1:2));
            end
        end
        
        function [pointSet, pointSetMin] = generateEdges(obj, backgroundIdx, edgeThreshold, backgroundThreshold)
            arguments
                obj ImageData
                backgroundIdx (1,1) double = 0
                edgeThreshold (1,1) double = 0.1
                backgroundThreshold (1,1) double = 127
            end
            narginchk(1, 4);
            pointSet = PointSet2D.empty(length(obj.imgs),0);
            pointSetMin = PointSet2D.empty(length(obj.imgs),0);
            imgEdge = cell(length(obj.imgs), 1);
            imgEdgeMin = cell(length(obj.imgs), 1);
            for i = 1 : length(obj.imgs)
                imgGray = rgb2gray(obj.imgs{i});
                imgBackground = uint8(ones(size(obj.imgs{i}(1:2))));
                if(backgroundIdx > 0)
                    imgNegative = uint8(abs(int8(obj.imgs{i}) - int8(obj.imgs{backgroundIdx})));
                    imgNegative = imgNegative(:,:,1).^2 + imgNegative(:,:,2).^2 + imgNegative(:,:,3).^2;
                    imgBackground = imgNegative >= backgroundThreshold;
                    imgBackground = uint8(255*imgBackground);
                elseif backgroundIdx == i
                    imgBackground = uint8(zeros(size(obj.imgs{i}(1:2))));
                end
                imgEdge{i} = edge(imgGray,'canny', edgeThreshold);
                imgEdgeMin{i} = imgEdge{i} & imgBackground;
                [edgeIdxsX, edgeIdxsY] = find(imgEdge{i});                
                % swap x and y to have x as horizontal and y as verical axis
                edgeIdxs = [edgeIdxsY edgeIdxsX];                 
                [edgeMinIdxsX, edgeMinIdxsY] = find(imgEdgeMin{i});
                % swap x and y to have x as horizontal and y as verical axis
                edgeMinIdxs = [edgeMinIdxsY edgeMinIdxsX];
                %{
                subplot(1,2,1);
                imshow(imgEdge);
                subplot(1,2,2);
                imshow(imgEdgeMin);
                %}
                points_m = ImageData.pixel2meter(edgeIdxs, obj.intrinsic);
                pointsMin_m = ImageData.pixel2meter(edgeMinIdxs, obj.intrinsic);
                pointSet(i) = PointSet2D(points_m);
                pointSetMin(i) = PointSet2D(pointsMin_m);
            end
        end
    end   
    %TODO edge threshold with slider
    %%{
    methods (Static)
        function points_m = pixel2meter(points_pix, intrinsic)
        %
        % Convert image pixel to meters using camera instrinsic parameters
        %
        points_m = (points_pix - intrinsic.PrincipalPoint') ./ intrinsic.FocalLength';
        end
        %{
        function edgeThreshold = edgeImage(imgGray, edgeThreshold)
            fig = uifigure;
            ax=uiaxes(fig);
            imgEdge = edge(imgGray,'canny', edgeThreshold);
            imshow(imgEdge,"Parent", ax);
            sld = uislider(fig,'Value',edgeThreshold, 'Position', [10 30 150, 3], 'Limits', [0 1],...
                'ValueChangedFcn',@(sld,event) updateEdgeThreshold(sld,edgeThreshold));
        end
        
        function updateEdgeThreshold(sld,edgeThreshold)
            edgeThreshold = sld.Value;
        end
        %}
    end

end

