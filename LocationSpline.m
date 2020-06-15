classdef LocationSpline < LocationRange
    properties (SetAccess = private)
        pp_ % 2D piecewise cubic polynomial
        dpp_% analytical derivative of pp_, 2D piecewise quadratic polynomial
    end
    methods
        function obj = LocationSpline(locs)
            arguments
                locs (2,:) double
            end
            n = size(locs, 2);
            uu = linspace(0,1,n);
            obj.pp_ = spline(uu,locs);
            obj.dpp_ = obj.pp_;
            obj.dpp_.coefs = obj.dpp_.coefs * [3 0 0; 0 2 0; 0 0 1; 0 0 0];
            obj.dpp_.order = 3;
        end
        
        function p = Loc(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            p = ppval(obj.pp_,u); % 2 x n matrix
        end
        
        function d = Normal(obj, u)
            t = obj.Tangent(u);
            d = zeros(size(t));
            for i = 1:length(u)
                d(:,i) = Unit2D([-t(2,i);t(1,i)]);
            end
        end
        
        function t = Tangent(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            t = ppval(obj.dpp_,u); % 2 x n matrix
        end
    end
end