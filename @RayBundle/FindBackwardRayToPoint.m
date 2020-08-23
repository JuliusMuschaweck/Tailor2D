function [u,ray, opl, ok] = FindBackwardRayToPoint(obj, p)
    [uu, rray, oopl, ok] = obj.FindRayToPoint(p);
    if ~ok && nargout < 4
        error('FindBackwardRayToPoint: no ray found');
    end
    if isscalar(uu)
        u = uu;
        ray = rray;
        opl = oopl;
        if oopl >= 0
            if nargout < 4
                error('FindBackwardRayToPoint: p ahead of ray.p_');
            else
                ok = false;
            end
        end
    else
        test = oopl < 0;
        u = uu(test);
        ray = rray(test);
        opl = oopl(test);
        if ~(any(test))
            if nargout < 4
                error('FindBackwardRayToPoint: all p ahead of ray.p_');
            else
                ok = false;
            end
        end
    end
end

