classdef Ellipse
    % Ellipse
    
    properties
        implicitEllipse ImplicitEllipse
        parametricEllipse ParametricEllipse
    end
    
    methods
        function out = Ellipse(obj)
            % Constructor
            if nargin == 1       
                if isa(obj,'Ellipse')
                    out.implicitEllipse = obj.implicitEllipse;
                    out.parametricEllipse = obj.parametricEllipse;
                elseif isa(obj,'ImplicitEllipse')   
                    out.implicitEllipse = ImplicitEllipse(obj);
                    parametricEllipse = implicitEllipseO2parametricEllipseO(obj);
                    out.parametricEllipse = ParametricEllipse(parametricEllipse);
                elseif isa(obj,'ParametricEllipse')
                    out.parametricEllipse = ParametricEllipse(obj);
                    implicitEllipse = parametricEllipseO2implicitEllipseO(obj);
                    out.implicitEllipse = ImplicitEllipse(implicitEllipse);
                elseif isvector(obj) && all(size(obj) == [1 6])
                    out.implicitEllipse = ImplicitEllipse(obj);
                    parametricEllipse = implicitEllipseO2parametricEllipseO(out.implicitEllipse);
                    out.parametricEllipse = ParametricEllipse(parametricEllipse);
                elseif isvector(obj) && all(size(obj) == [1 5])
                    out.parametricEllipse = ParametricEllipse(obj);
                    implicitEllipse = parametricEllipseO2implicitEllipseO(out.parametricEllipse);
                    out.implicitEllipse = ImplicitEllipse(implicitEllipse);
                end
            end
        end
    end
end

