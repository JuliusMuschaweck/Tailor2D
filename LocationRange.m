classdef LocationRange < Range
    methods (Abstract)
        p = Loc(obj, u)
        d = Normal(obj, u)
        t = Tangent(obj, u)
    end
    
    methods
        function [px,py] = Points(obj, nPts) % list of coordinates to draw full line
            arguments
                obj
                nPts (1,1) double {mustBeInteger, mustBeGreaterThan(nPts, 1)}
            end
            tmp = obj.Loc(linspace(obj.umin_, obj.umax_, nPts));
            px = tmp(1,:);
            py = tmp(2,:);
        end
        
        function [u, loc, normal, ok] = Intersect(obj, ray) % returns u and loc and normal where ray intersects 
            arguments
                obj
                ray Ray
            end
            test0 = obj.TestIntersect(obj.umin_, ray); 
            test1 =  obj.TestIntersect(obj.umax_, ray);
            test = test0 * test1;
            ok = true;
            if test < 0 % bracket ok
                func = @(uu) obj.TestIntersect(uu, ray);
                u = FindRoot1D(func, obj.umin_, obj.umax_);
            else
                if abs(test0) < eps
                    u = obj.umin_;
                elseif abs(test1) < eps
                    u = obj.umax_;
                else
                    ok = false;
                end
            end
            if ok
                loc = obj.Loc(u);
                if nargout > 2
                    normal = obj.Normal(u);
                end
            else
                if nargout > 3
                    u = NaN;
                    loc = [NaN;NaN];
                    normal = [NaN;NaN];
                else
                    error('LocationRange.Intersect: No intersection found');
                end
            end
        end
        
        function [rv, u] = Distance(obj, point)
            % find normal through point, return signed distance from obj to point along this normal
            arguments
                obj
                point (2,1) double
            end
            test = obj.TestPointLeftRight( obj.umin_, point) * obj.TestPointLeftRight( obj.umax_, point);
            if test < 0 % bracket ok
                func = @(uu) obj.TestPointLeftRight(uu, point);
                u = FindRoot1D(func, obj.umin_, obj.umax_);
                rv = (point - obj.Loc(u))' * obj.Normal(u);
            else
                error('LocationRange.Distance: No normal through point found');
            end
        end
    end
    methods (Access = private)
        function rv = TestIntersect(obj, u, ray)
            rv = Cross2D(obj.Loc(u) - ray.p_, ray.k_);
        end
        function rv = TestPointLeftRight(obj, u, point)
            rv = Cross2D(point - obj.Loc(u), obj.Normal(u));
        end
    end
end