%% Ray class
% A container for a 2D ray, with location and direction.
%% Description
% This class has two 2D column vector properties: location |p_| and and direction |k_|. While location can be anything,
% the length of |k_| is taken to be the refractive index. A |Ray| also knows its closest distance to a
% point, and knows where it intersects another ray.
%% Examples
%% Input Arguments
% * |x|: scalar double. Length of something
%% Output Arguments
% * |rv|: scalar double. Width of something
%% Algorithm
% Does something
classdef Ray
    properties
        p_ (2,1) double = [0;0]
        k_ (2,1) double = [1;0]
    end
    methods
        function obj = Ray(p, k)
            if nargin > 0
                obj.p_ = p;
            end
            if nargin > 1
                obj.k_ = k;
            end
        end
        
        function rv = SignedDistanceToPoint(obj, point)
            arguments
                obj
                point (2,1) double
            end
            % positive if point is left of ray
            rv = Cross2D(Unit2D(obj.k_), point - obj.p_);
        end
        
        function [rv, lambda, mu] = IntersectRay( obj, r )
            % rv == obj.p_ + lambda * obj.k_ == r.p_ + mu * r.k_
            arguments
                obj
                r (1,1) Ray
            end
            A = [obj.k_, - r.k_];
            D = A(1,1) * A(2,2) - A(2,1) * A(1,2);
            if abs(D) == 0
                rv = NaN(2,1);
                lambda = NaN;
                mu = NaN;
                return;
            end
            rhs = r.p_ - obj.p_;
            iA = [A(2,2), -A(1,2); -A(2,1), A(1,1)] / D;
            lm = iA * rhs;
            lambda = lm(1);
            mu = lm(2);
            rv = obj.p_ + lambda * obj.k_;
        end
        
        function ray = RefractAtPoint(obj, p, normal, n_after) % precondition: p is on ray
            arguments
                obj
                p (2,1) double
                normal (2,1) double
                n_after (1,1) double {mustBePositive}
            end
            in = obj.k_;
            ni = obj.n_refr();
            no = n_after;
            n = normal;
            if in' * n < 0
                n = -n;
            end
            D = no^2 - ni^2 + (in' * n)^2;
            if D < 0
                error('Ray.RefractAtPoint: TIR');
            end
            out = in + (sqrt(D) - in' * n) * n;
            ray = Ray(p, out);
        end
        
        function rv = n_refr(obj)
            rv = Norm2D(obj.k_);
        end
    end
end