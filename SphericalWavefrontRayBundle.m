classdef SphericalWavefrontRayBundle < RayBundle
    properties (SetAccess = private)
        c_ (2,1) double % sphere center
        r_ (1,1) double % radius
        theta0_ (1,1) double % start angle. theta_1 > theta_0 means outward, else inward
        theta1_ (1,1) double % end angle
        n_ (1,1) double % refractive index
    end
    methods
        function obj = SphericalWavefrontRayBundle(c, r, theta0, theta1, n)
            arguments
                c (2,1) double % circle center
                r (1,1) {double, mustBeNonnegative} % radius
                theta0 (1,1) double % start angle
                theta1 (1,1) double % end angle
                n (1,1) {double, mustBePositive} = 1 % refractive index
            end
            if theta0 == theta1
                error('SphericalWavefrontRayBundle: angles must not be equal');
            end
            obj.c_ = c;
            obj.r_ = r;
            obj.theta0_ = theta0;
            obj.theta1_ = theta1;
            obj.n_ = n;
        end
        
        function r = Ray(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            [~, st, ct] = obj.Theta(u);
            loc = obj.c_ + obj.r_ * [ct;st];
            dir = obj.n_ * [ct; st] * obj.reverse_;
            if obj.theta0_ > obj.theta1_ % go inside
                dir = - dir;
            end
            if isscalar(u)
                r = Ray(loc, dir);
            else
                r = repmat(Ray(),1,length(u));
                for i = 1:length(u)
                    r(i) = Ray(loc(:,i), dir(:,i));
                end
            end
        end
        
        function p = Loc(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            [~, st, ct] = obj.Theta(u);
            p = obj.c_ + obj.r_ * [ct;st];
        end
        
        function d = Normal(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            [~, st, ct] = obj.Theta(u);
            d = [ct;st];
        end
        
        function t = Tangent(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            [~, st, ct] = obj.Theta(u);
            fac = (obj.theta1_ - obj.theta0_) * obj.r_ / obj.du_; % takes care of left/right
            t = fac * [-st; ct];
        end
        
        function k = iDir(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            [~, st, ct] = obj.Theta(u);
            if obj.theta0_ > obj.theta1_ % go outside
                k = obj.n_ * [-ct; -st];
            else
                k = obj.n_ * [ct; st];
            end
        end
        
        function dk = idkdu(obj, u)
            obj.CheckBounds(u); % u is 1 x n row vector
            [~, st, ct] = obj.Theta(u);
            fac = obj.n_ * (obj.theta1_ - obj.theta0_) / obj.du_; % takes care of left/right
            dk = fac * [-st; ct];
        end
        
        function rv = Energy(obj,u) 
            error ('not implemented');
        end
        
    end
    methods (Access = private)
        function [rv, sint, cost] = Theta(obj, u)
            rv = obj.LinInterpol(u, obj.theta0_, obj.theta1_);
            if nargout > 1
                sint = sin(rv);
            end
            if nargout > 2
                cost = cos(rv);
            end
        end
    end
end        
