classdef LocationArc < LocationRange
    properties (SetAccess = private)
        c_ (2,1) double
        r_ (1,1) {double, mustBePositive} = 1
        theta_0_ (1,1) double
        theta_1_ (1,1) double
    end
    
    methods 
        function obj = LocationArc(c, r, theta_0, theta_1)
            obj.c_ = c;
            obj.r_ = r;
            obj.theta_0_ = theta_0;
            obj.theta_1_ = theta_1;
        end
        function p = Loc(obj, u)
            obj.CheckBounds(u); % postcondition: u is scalar or row vector
            theta = obj.LinInterpol(u, obj.theta_0_, obj.theta_1_);% same for theta
            p = obj.c_ + obj.r_ * [cos(theta); sin(theta)]; % yes, works fine for row vector u!
        end
        function d = Normal(obj, u)
            obj.CheckBounds(u);
            theta = obj.LinInterpol(u, obj.theta_0_, obj.theta_1_);
            if obj.theta_0_ <= obj.theta_1_
                d = [cos(theta); sin(theta)];
            else
                d = - [cos(theta); sin(theta)];
            end
        end
        function t = Tangent(obj, u)
            obj.CheckBounds(u);
            theta = obj.LinInterpol(u, obj.theta_0_, obj.theta_1_);
            cost = cos(theta);
            sint = sin(theta);
            fac = obj.r_ * (obj.theta_1_ - obj.theta_0_) / obj.du_;
            t = fac * [-sint; cost];
        end
    end    
end