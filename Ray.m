%% Ray class
% A container for a 2D ray, with location and direction.
%% Description
% This class has two 2D column vector properties: location |p_| and and direction |k_|. While location can be anything,
% the length of |k_| is taken to be the refractive index. A |Ray| also knows its closest distance to a
% point, knows where it intersects another ray, and knows where it intersects a <file:LocationRange.html,
% LocationRange>.
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
    end
end