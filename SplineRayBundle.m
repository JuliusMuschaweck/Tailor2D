classdef SplineRayBundle < RayBundle
    properties (SetAccess = private)
        pp_ % 5D piecewise cubic polynomial
        dpp_% analytical derivative of pp_, 5D piecewise quadratic polynomial
    end
    methods
        function obj = SplineRayBundle(locs, dirs, energies)
            arguments
                locs (2,:) double
                dirs (2,:) double
                energies (1,:) double
            end
            n = size(locs, 2);
            assert(size(dirs,2) == n, 'SplineRayBundle: size mismatch: %g locations but %g directions',n,size(dirs,2));
            if isempty(energies)
                energies = linspace(0,1,n);
            else
                assert(length(energies) == n, 'SplineRayBundle: size mismatch: %g locations but %g energies',n,length(irrads));
            end
            uu = linspace(0,1,n);
            obj.pp_ = spline(uu,cat(1,locs, dirs, energies));
            obj.dpp_ = obj.pp_;
            obj.dpp_.coefs = obj.dpp_.coefs * [3 0 0; 0 2 0; 0 0 1; 0 0 0];
            obj.dpp_.order = 3;
        end
        
        function r = Ray(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            tmp = ppval(obj.pp_,u); % 5 x n matrix
            if isscalar(u)
                r = Ray(tmp(1:2), tmp(3:4));
            else
                r = repmat(Ray(),1,length(u));
                for i = 1:length(u)
                    r(i) = Ray(tmp(1:2,i), tmp(3:4,i));
                end
            end
        end
        
        function p = Loc(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            tmp = ppval(obj.pp_,u); % 5 x n matrix
            p = tmp(1:2,:);
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
            tmp = ppval(obj.dpp_,u); % 5 x n matrix
            t = tmp(1:2,:);
        end
        
        function k = Dir(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            tmp = ppval(obj.pp_,u); % 5 x n matrix
            k = tmp(3:4,:);
        end
        
        function dk = dkdu(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            tmp = ppval(obj.dpp_,u); % 5 x n matrix
            dk = tmp(3:4,:);
        end
        
        function rv = Energy(~,u) 
            obj.CheckBounds(u); % u is 1 x n row vector
            tmp = ppval(obj.pp_,u); % 5 x n matrix
            rv = tmp(5,:);
        end
        
    end
end