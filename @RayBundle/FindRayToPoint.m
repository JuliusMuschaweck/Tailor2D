function [u, ray, opl, ok] = FindRayToPoint(obj, p, nameValueArgs) % a RayBundle method
    arguments
        obj
        p (2,1) double
        nameValueArgs.doExtend (1,1) logical = false
    end
    ok = true;
    % fprintf('FindRayToPoint [%g;%g]\n',p(1),p(2));
    testmin = obj.TestIntersect(obj.umin_, p);
    testmax = obj.TestIntersect(obj.umax_, p);
    if testmin * testmax < 0 % bracket ok
        [u, ray, opl] = FindBracketedRayToPoint(obj, p, obj.umin_, obj.umax_);
        return;
    else
        if abs(testmin) < eps
            u = obj.umin_;
            ray = obj.Ray(u);
            opl = (p - ray.p_)' * ray.k_;
            return;
        elseif abs(testmax) < eps
            u = obj.umax_;
            ray = obj.Ray(u);
            opl = (p - ray.p_)' * ray.k_;
            return;
        end % now testmin * testmax > 0, p may still be near caustic, expect two intersections
        if testmin > 0 % both positive, look for minimum
            testfun = @(uu) obj.TestIntersect(uu, p);
        else % both negative, look for maximum
            testfun = @(uu) - obj.TestIntersect(uu, p);
        end
        [uu, fval] = fminbnd(testfun, obj.umin_, obj.umax_,optimset('OutputFcn',@myoutput,'Display','off'));
        if fval == 0 % point on caustic, single solution
            u = uu;
            ray = obj.Ray(u);
            opl = (p - ray.p_)' * ray.k_;
        elseif fval < 0 % two solutions
            [u, ray, opl] = FindBracketedRayToPoint(obj, p, obj.umin_, uu);
            [u(2), ray(2), opl(2)] = FindBracketedRayToPoint(obj, p, uu, obj.umax_);
            if opl(2) < opl(1)
                u = fliplr(u);
                ray = fliplr(ray);
                opl = fliplr(opl);
            end
        else % fval > 0, no solution
            if nameValueArgs.doExtend 
                % try extension at umin_, by 1 u unit 
                p0 = obj.Loc(obj.umin_);
                k0 = obj.Dir(obj.umin_);
                t0 = obj.Tangent(obj.umin_);
                ext0 = StraightLineCollimatedRayBundle(p0, p0 - t0, k0, obj.umin_-1, obj.umin);
                [uu, rr, oopl, ook] = ext0.FindRayToPoint(p);
                if ook
                    
                else
                end
            end
            if nargout < 4
                error('RayBundle.FindRayToPoint: No intersection found');
            else
                u = NaN;
                ray = Ray([NaN;NaN], [NaN;NaN]);
                opl = NaN;
                ok = false;
            end
        end
    end
    function stop = myoutput(x, optimValues, state)
        if isempty(optimValues.fval)
            stop = false;
        else
            stop = optimValues.fval < 0;
        end
    end
end