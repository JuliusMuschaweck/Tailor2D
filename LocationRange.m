classdef LocationRange < Range
    methods (Abstract)
        p = Loc(obj, u)
        d = Normal(obj, u)
        t = Tangent(obj, u)
    end
    
    methods
        function [u, loc] = Intersect(obj, ray) % returns u and loc where ray intersects 
            arguments
                obj
                ray Ray
            end
            test = obj.TestIntersect(obj.umin_, ray) * obj.TestIntersect(obj.umax_, ray);
            if test < 0 % bracket ok
                func = @(uu) obj.TestIntersect(uu, ray);
                u = FindRoot1D(func, obj.umin_, obj.umax_);
                loc = obj.Loc(u);
            else
                error('LocationRange.Intersect: No intersection found');
            end
        end
        
    end
    methods (Access = private)
        function rv = TestIntersect(obj, u, ray)
            rv = Cross2D(obj.Loc(u) - ray.p_, ray.k_);
        end
    end
end