%% Cross2D
% Computes the (scalar) cross product of two 2D vectors
%% Syntax
% |rv = Cross2D(lhs,rhs)|
%% Description
% |rv = Cross2D(lhs,rhs)| computes the (scalar) cross product of |lhs| and |rhs|
%% Examples
% |Cross2D([2;0],[0;2])|
% returns |4|. When |rhs| points towards the left side of |lhs|, result is positive, else negative
%% Input Arguments
% * |lhs|: 2D column vector.
% * |rhs|: 2D column vector.
%% Output Arguments
% * |rv|: scalar double.
%% Algorithm
% Computes |lhs(1)*rhs(2) - lhs(2)*rhs(1)|.
function rv = Cross2D(lhs,rhs)
    arguments
        lhs (2,1) double
        rhs (2,1) double
    end
    rv = lhs(1)*rhs(2) - lhs(2)*rhs(1);
end