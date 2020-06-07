%% Class Ray
% A container for a 2D ray, with location and direction.
%% Syntax
% |rv = func(x)|
%% Description
% |rv = func(x)| computes rv
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