classdef LocationStraight < LocationRange
    properties (SetAccess = private)
        p0_ (2,1) double
        p1_ (2,1) double
    end
    
    methods 
        function obj = LocationStraight(p0, p1)
            obj.p0_ = p0;
            obj.p1_ = p1;
            if norm(p1-p0) == 0
                error('LocationLine error: equal end points: [%g;%g] == [%g;%g]',p0(1),p0(2),p1(1),p1(2));
            end
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
    end    
end