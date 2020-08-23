function [p, lambda] = CausticPoint(obj, u)
    % caustic point p == obj.p_ + lambda * Unit2D(obj.k_)
    dk = obj.dkdu(u);
    dk2 = dk' * dk;
    if dk2 < eps
        lambda = Inf;
        p = [Inf;Inf];
        return;
    end
    dp = obj.Tangent(u);
    r = obj.Ray(u);
    lambda = - (dp' * dk) / dk2;
    p = r.p_ + lambda * Unit2D(r.k_);
end
