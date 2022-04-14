classdef SphericalProjection
    % Spherical Projection
    
    properties
        points  PointSet2D
        inliers PointSet2D
        ellipse Ellipse
        sphere  Sphere
    end
    
    methods
        function out = SphericalProjection(obj, r, isRobust)
            % Constructor
            arguments
                obj 
                r (1,1) double = 1.0        %sphere radius
                isRobust (1,1) logical = 1  % 1: robust problem with outliers 
                                            % 0: standard problem only with inlier
            end
            if nargin >= 1       
                if isa(obj,'SphericalProjection')
                    out.points = obj.points;
                    out.inliers = obj.inliers;
                    out.ellipse = obj.ellipse;
                    out.sphere = obj.sphere;
                elseif isa(obj,'PointSet2D')   
                    if isRobust
                        out.points = obj;
                        out.inliers = PointSet2D();
                        out.ellipse = Ellipse();
                        out.sphere = Sphere();
                    else
                        out.points = obj;
                        out.inliers = obj;
                        out.ellipse = fitEllipse(obj);
                        out.sphere = fitSphere(obj, r);
                    end
                end
            end
        end
    end
end

