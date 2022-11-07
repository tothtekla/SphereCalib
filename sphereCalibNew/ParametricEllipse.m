classdef ParametricEllipse
    % Parametric Ellipse
    
    properties 
        parameters (1,5) double
    end
    properties (Dependent)
        a (1,1) double
        b (1,1) double
        e0 (1,2) double % = [ex ey]
        theta (1,1) double
    end
    
    methods
        function a = get.a(obj)
            a = obj.parameters(1);
        end
        function obj = set.a(obj, val)
            obj.parameters(1) = val;
        end
        
        function b = get.b(obj)
            b = obj.parameters(2);
        end
        function obj = set.b(obj, val)
            obj.parameters(2) = val;
        end
        
        function e0 = get.e0(obj)
            e0 = obj.parameters(3:4);
        end
        function obj = set.e0(obj, val)
            obj.parameters(3:4) = val;
        end
        
        function theta = get.theta(obj)
            theta = obj.parameters(5);
        end
        function obj = set.theta(obj, val)
            obj.parameters(5) = val;
        end
        
        function out = ParametricEllipse(obj)
            % Constructor
            if nargin == 0
                return; 
            end
            if isa(obj,'ParametricEllipse')
                out.parameters = obj.parameters;
            elseif isvector(obj) && all(size(obj) == [1 5])
                out.parameters = obj;
            end
        end
        
       function iE = parametricEllipseO2implicitEllipseO(pE)
            %
            %  Conversion from parametric ellipse object to implicit ellipse object 
            %
            arguments
                pE (1,1) ParametricEllipse
            end
            [A, B, C, D, E, F] = ParametricEllipse.parametricEllipse2implicitEllipse(pE.a, pE.b, pE.e0(1), pE.e0(2), pE.theta);
            iE = ImplicitEllipse([A, B, C, D, E, F]);
       end

       function s = parametricEllipseO2SphereO(pE, r)
            %
            %  Conversion from parametric ellipse object to sphere object 
            %
            arguments
                pE (1,1) ParametricEllipse
                r  (1,1) double
            end
            s0 = ParametricEllipse.parametricEllipse2SphereCenter(pE.a, pE.b, pE.e0(1), pE.e0(2), pE.theta, r);
            s = SphereConverter([s0 r]);
       end

       function pE = convertToPixel(pE, u0, v0, fu, fv)
            arguments
                pE (1,1) ParametricEllipse
                u0 (1,1) double
                v0 (1,1) double
                fu (1,1) double
                fv (1,1) double
            end
            [pE.a, pE.b] =  axesInMeter2axesInPixel(pE, fu, fv);
            pE.e0 = (pE.e0 .* [fu fv]) + [u0 v0];
            % TODO if fu ~= fv, theta is changing
       end

       function pts = generateEllipsePoints(pE, numInliers)
            %
            % Generate points around the ellipse contour at the origin with natural  
            % parametriztion, then apply rotation and translation
            %
            % Chebfun download: https://www.chebfun.org/download/
            %
            arguments
                pE (1,1) ParametricEllipse
                numInliers (1,1) double = 25
            end           
            % Set the interval of parameter t 
            t = chebfun('t',[0 2*pi]);
            % Construate the ellipse with a,b using complex numbers 
            ellipse = chebfun(pE.a*cos(t)+1i*pE.b*sin(t));
            % Find the natural parameterization
            natural = ellipse(inv(cumsum(abs(diff(ellipse)))));
            % Generate inliers in the origin-centred ellipse 
            inliersAtO_cmplx = natural(linspace(natural.domain(1),natural.domain(2),numInliers+1));
            % Convert complex numbers into homogeneus coord.-s. 
            % - Left out the last one (Same as the first)
            inliersAtO_hom = [real(inliersAtO_cmplx(1:end-1)) ; imag(inliersAtO_cmplx(1:end-1)) ; ones(1,numInliers)];
            % Transform the ellipse
            % Create rotation + translation matrix
            T = [cos(pE.theta), -sin(pE.theta),     pE.e0(1);...
                 sin(pE.theta), cos(pE.theta), pE.e0(2);...
                 0,              0,              1];
            inliers_hom = (T * inliersAtO_hom)';
            % Convert from homogeneus to cartesian coord. 
            pts = hom2cart(inliers_hom);
       end

        function [a_dist_pix, b_dist_pix] = axesInMeter2axesInPixel(pE, fu, fv)
            %
            % Convert a distance in the image from meters to pixels using camera instrinsic parameters
            % dist_pix: end point of the line segment
            % angle: rotation of the line segment
            arguments
                pE (1,1) ParametricEllipse
                fu (1,1) double
                fv (1,1) double
            end
            a_m = [pE.a zeros(length(pE.a), 1)];
            b_m = [zeros(length(pE.b), 1) pE.b];

            R = [ cos(pE.theta), -sin(pE.theta);
              sin(pE.theta), cos(pE.theta)];
            a_m(1:2) = (R * a_m(1:2)')';
            b_m(1:2) = (R * b_m(1:2)')';
           
            a_pix = a_m .* [fu fv];
            b_pix = b_m .* [fu fv];

            a_dist_pix = vecnorm(a_pix,2,2);
            b_dist_pix = vecnorm(b_pix,2,2);
        end

    end
    
    methods (Static)
        function [A, B, C, D, E, F] = parametricEllipse2implicitEllipse(a, b, ex, ey, theta)
            %
            % Conversion from parametric ellipse
            %              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
            %                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey
            %            to implicit ellipse 
            %              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
            %
            A = a^2 * sin(theta)^2 + b^2 * cos(theta)^2;
            B = 2*(b^2 - a^2) * sin(theta) * cos(theta);
            C = a^2*cos(theta)^2 + b^2*sin(theta)^2;
            D = -2*A*ex - B*ey ;
            E = -B*ex - 2*C*ey;
            F = A*ex^2 + B*ex*ey + C*ey^2 - a^2*b^2;
        end
        
        function s0 = parametricEllipse2SphereCenter(a, b, ex, ey, theta, r)
            %
            % Conversion from projected parametric ellipse     
            %              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
            %                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey  
            %            to implicit sphere             
            %              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
            % 

            % Get unit vectors directed to major ellipse axis enging points
            T = [a*cos(theta), -b*sin(theta), ex;...
                 a*sin(theta),  b*cos(theta), ey;...
                 0,               0,          1];
            a1 = (T * [-1, 0, 1]')';
            a2 = ( T * [1 , 0, 1]')';
            q_a1 = a1 / norm(a1);
            q_a2 = a2 / norm(a2);

            % Angle bisector
            w = (q_a2 + q_a1) / 2;
            w = w / norm(w);

            % Cone angle and direction
            cos2alpha = q_a1*q_a2';
            d = sqrt(2)*r / sqrt(1-cos2alpha);

            s0 = d * w;
        end
    end
end

