classdef RayBundle < Range
    properties (Access = protected)
        reverse_ = 1;
    end
    methods (Abstract)
        r = Ray(obj, u)
        p = Loc(obj, u)
        d = Normal(obj, u)
        t = Tangent(obj, u)
        k = iDir(obj, u) % unreversed Dir
        dk = idkdu(obj, u) % unreversed dkdu
    end
    
    methods (Sealed = true)
        function k = Dir(obj, u) 
            k = obj.iDir(u) * obj.reverse_;
        end
        function dk = dkdu(obj, u) 
            dk = obj.idkdu(u) * obj.reverse_;
        end
    end
    
    methods
        function rb = Reverse(obj)
            if nargout == 1
                rb = copy(obj);
                rb.reverse_ = -1;
            else
                obj.reverse_ = -1;
            end
        end
        function rb = UnReverse(obj)
            if nargout == 1
                rb = copy(obj);
                rb.reverse_ = 1;
            else
                obj.reverse_ = 1;
            end
        end
        function rb = FlipReverse(obj)
            if nargout == 1
                rb = copy(obj);
                rb.reverse_ = rb.reverse_;
            else
                obj.reverse_ = - obj.reverse_;
            end
        end
        
        
        rv = Energy(~,u)
        
        function rv = isWavefront(~)
            rv = false;
        end
        
        [u,ray, opl, ok] = FindForwardRayToPoint(obj, p)
        
        [u,ray, opl, ok] = FindBackwardRayToPoint(obj, p)

        [u, ray, opl, ok] = FindRayToPoint(obj, p)
        
        rv = PropagateTo(obj, locationRange, nameValueArgs)
        
        [p, lambda] = CausticPoint(obj, u)
        
    end
    methods (Access = private)
        function rv = TestIntersect(obj, u, p) % > 0 if p is left of ray(u)
            ray = obj.Ray(u);
            rv = Cross2D(p - ray.p_, ray.k_);
        end
        function [u, ray, opl] = FindBracketedRayToPoint(obj, p, u0, u1)
            func = @(uu) obj.TestIntersect(uu, p);
            u = FindRoot1D(func, u0, u1);
            ray = obj.Ray(u);
            opl = (p - ray.p_)' * ray.k_;
        end
    end
    
end