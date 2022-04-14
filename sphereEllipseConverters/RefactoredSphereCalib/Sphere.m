classdef Sphere
    % Sphere
    
    properties
        parameters (1,4) double
    end
    
    properties (Dependent)
        s0 (1,3) double % = [x0 y0 z0]
        r (1,1) double
    end
    
    methods
        function c = get.s0(obj)
            c = obj.parameters(1:3);
        end
        function obj = set.s0(obj, val)
            obj.parameters(1:3) = val;
        end
        
        function r = get.r(obj)
            r = obj.parameters(4);
        end
        function obj = set.r(obj, val)
            obj.parameters(4) = val;
        end
        
        function out = Sphere(obj)
            % Constructor
            if nargin == 0
                return; 
            end
            if isa(obj,'Sphere')
                out.parameters = obj.parameters;
            elseif isvector(obj) && all(size(obj) == [1 4])
                out.parameters = obj;
            end
        end
        
        function iE = sphereO2implicitEllipseO(s)
            %
            %  Conversion from sphere object to implicit ellipse object 
            %
            arguments
                s (1,1) Sphere
            end
            [A, B, C, D, E, F] = Sphere.sphere2implicitEllipse(s.s0(1), s.s0(2), s.s0(3), s.r);
            iE = ImplicitEllipse([A, B, C, D, E, F]);
        end
        
        function pE = sphereO2parametricEllipseO(s)
            %
            %  Conversion from sphere object to parametric ellipse object 
            %
            arguments
                s (1,1) Sphere
            end
            [a, b, ex, ey, theta] = Sphere.sphere2parametricEllipse(s.s0(1), s.s0(2), s.s0(3), s.r);
            pE = ParametricEllipse([a, b, ex, ey, theta]);
        end
    end
    
    methods (Static)
        function [A, B, C, D, E, F] = sphere2implicitEllipse(x0, y0, z0, r)
            %
            % Conversion from implicit sphere             
            %              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
            %            to projected implicit ellipse     
            %              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
            %
            % Based on Tekla Tóth, Zoltán Pusztai, Levente Hajder: Automatic LiDAR-Camera 
            % Calibration of Extrinsic Parameters Using a Spherical Target., ICRA 2020
            %
            A = y0^2+z0^2-r^2;
            B = -2*x0*y0;
            C = x0^2+z0^2-r^2;
            D = -2*x0*z0;
            E = -2*y0*z0;
            F = x0^2+y0^2-r*r;
        end
        
        function [a, b, ex, ey, theta] = sphere2parametricEllipse(x0, y0, z0, r)
            %
            % Conversion from implicit sphere             
            %              s(x, y, z): (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = r^2
            %            to projected parametric ellipse     
            %              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
            %                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey  
            %
            a = r*sqrt(x0^2 + y0^2 + z0^2 - r^2)/(z0^2 - r^2);
            b = r/sqrt(z0^2 - r^2);
            ex = x0*z0/(z0^2 - r^2);
            ey = y0*z0/(z0^2 - r^2);
            if y0 == 0
                theta = 0;
            elseif x0 == 0
                theta = 0.5 * pi;
            else
                theta = atan(y0/x0);
            end
        end
    end 
end

