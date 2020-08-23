classdef StraightLineCollimatedRayBundle < RayBundle
    properties (SetAccess = private)
        p0_ % first end point
        p1_ % second end point
        k_ % common direction of all rays
    end
    methods
        function obj = StraightLineCollimatedRayBundle(p0, p1, k, umin, umax)
            arguments
                p0 (2,1) double % 
                p1 (2,1) double % 
                k (2,1) double % 
                umin (1,1) double
                umax (1,1) double
            end
            obj.p0_ = p0;
            obj.p1_ = p1;
            obj.k_ = k;
            if ~(umin < umax)
                error('StraightLineCollimatedRayBundle: umin not < umax (%g, %g)',umin, umax);
            end
            obj.umin_ = umin;
            obj.umax_ = umax;
        end
        
        function r = Ray(obj, u)
            r = Ray(obj.Loc(u), obj.k_);
        end
        
        function p = Loc(obj, u)
            obj.CheckBounds(u);
            p = obj.LinInterpol(u, obj.p0_, obj.p1_);
        end
        function d = Normal(obj, u)
            obj.CheckBounds(u);
            tmp = obj.p1_ - obj.p0_;
            d = Unit2D([tmp(2); - tmp(1)]);
            if ~isscalar(u)
                d = repmat(d, length(u), 1);
            end
        end
        function t = Tangent(obj, u)
            obj.CheckBounds(u);
            t = (obj.p1_ - obj.p0_) / obj.du_;
            if ~isscalar(u)
                t = repmat(t, length(u), 1);
            end
        end
        
        function k = iDir(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            k = obj.k_;
            if ~isscalar(u)
                k = repmat(k, length(u), 1);
            end
        end
        
        function dk = idkdu(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            dk = [0;0];
            if ~isscalar(u)
                dk = repmat(k, length(u), 1);
            end
        end
        
        function rv = Energy(~,~) 
            %rv = NaN;
            error ('not implemented');
        end
        
    end
end