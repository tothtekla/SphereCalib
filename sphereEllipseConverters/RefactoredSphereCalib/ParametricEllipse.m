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
            s = Sphere([s0 r]);
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

