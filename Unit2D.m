function [rv, n] = Unit2D(rhs)
    arguments
        rhs (2,1) double
    end
    n = Norm2D(rhs);
    if n == 0
        error('Unit2D: |rhs| == 0');
    end
    rv = rhs / n;
end