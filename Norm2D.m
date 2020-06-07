function rv = Norm2D(rhs)
    arguments
        rhs (2,1) double
    end
    rv = sqrt(rhs(1)^2+rhs(2)^2);
end