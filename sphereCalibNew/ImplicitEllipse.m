classdef ImplicitEllipse
    % Implcit Ellipse
    
    properties 
        coefficients (1,6) double
    end
    properties (Dependent)
        A (1,1) double
        B (1,1) double
        C (1,1) double
        D (1,1) double
        E (1,1) double
        F (1,1) double
    end
    
    methods
        function A = get.A(obj)
            A = obj.coefficients(1);
        end
        function obj = set.A(obj, val)
            obj.coefficients(1) = val;
        end
        
        function B = get.B(obj)
            B = obj.coefficients(2);
        end
        function obj = set.B(obj, val)
            obj.coefficients(2) = val;
        end
        
        function C = get.C(obj)
            C = obj.coefficients(3);
        end
        function obj = set.C(obj, val)
            obj.coefficients(3) = val;
        end
        
        function D = get.D(obj)
            D = obj.coefficients(4);
        end
        function obj = set.D(obj, val)
            obj.coefficients(4) = val;
        end
        
        function E = get.E(obj)
            E = obj.coefficients(5);
        end
        function obj = set.E(obj, val)
            obj.coefficients(5) = val;
        end
        
        function F = get.F(obj)
            F = obj.coefficients(6);
        end
        function obj = set.F(obj, val)
            obj.coefficients(6) = val;
        end      
        
        function out = ImplicitEllipse(obj)
            % Constructor
            if nargin == 0
                return; 
            end
            if isa(obj,'ImplicitEllipse')
                out.coefficients = obj.coefficients;
            elseif isvector(obj) && all(size(obj) == [1 6])
                obj = obj/norm(obj);
                out.coefficients = obj;
            end
        end
        
        function obj = normalize(obj)
            arguments
                obj ImplicitEllipse
            end
            obj.coefficients = obj.coefficients / norm(obj.coefficients);
        end
        
        function pE = implicitEllipseO2parametricEllipseO(iE)
            %
            %  Conversion from implicit ellipse object to parametric ellipse object 
            %
            arguments
                iE (1,1) ImplicitEllipse
            end
            [a, b, ex, ey, teta] =  ImplicitEllipse.implicitEllipse2parametricEllipse(iE.A, iE.B, iE.C, iE.D, iE.E, iE.F);
            pE = ParametricEllipse([a, b, ex, ey, teta]);
        end
    end
    
    methods (Static)
        function [a, b, ex, ey, theta] = implicitEllipse2parametricEllipse(A, B, C, D, E, F)
            %
            % Conversion from implicit ellipse
            %              e(u, v)   : A*u^2 + B*u*v + C*v^2 + D*u + E*v + F = 0 
            %            to parametric ellipse  
            %              e(t)      : a*cos(t)*cos(theta) - b*sin(t)*sin(theta) + ex
            %                          a*cos(t)*sin(theta) + b*sin(t)*cos(theta) + ey 
            %
            arguments
                A (1,1) double
                B (1,1) double
                C (1,1) double
                D (1,1) double
                E (1,1) double
                F (1,1) double
            end
            Disc = B^2 - 4*A*C;
            a = (-sqrt(2*(A*E^2 + C*D^2 - B*D*E + Disc*F)*((A + C) + sqrt((A - C)^2 + B^2)))) / Disc;
            b = (-sqrt(2*(A*E^2 + C*D^2 - B*D*E + Disc*F)*((A + C) - sqrt((A - C)^2 + B^2)))) / Disc;
            ex = (2*C*D - B*E) / Disc;
            ey = (2*A*E - B*D) / Disc;
            if (B == 0)
                if (A < C)
                    theta = 0;
                else
                    theta = 0.5 * pi;
                end
            else
                theta = atan(1/B * (C - A - sqrt((A - C)^2 + B^2)));
            end
            
        end
    end
end

