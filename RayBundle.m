classdef RayBundle < Range
    methods (Abstract)
        r = Ray(obj, u)
        p = Loc(obj, u)
        d = Normal(obj, u)
        t = Tangent(obj, u)
        k = Dir(obj, u)
        dk = dkdu(obj, u)
    end
    
    methods
        function rv = Energy(~,u) 
            CheckBounds(u);
            rv = u;
        end
        function rv = isWavefront(~)
            rv = false;
        end
        
        function [u,ray, opl, ok] = FindForwardRayToPoint(obj, p)
            [uu, rray, oopl, ok] = obj.FindRayToPoint(p);
            if ok < 0 && nargout < 4
                error('FindForwardRayToPoint: no ray found');
            end
            if isscalar(uu)
                u = uu;
                ray = rray;
                opl = oopl;
                if oopl <= 0
                    if nargout < 4
                        error('FindForwardRayToPoint: p behind ray.p_');
                    else 
                        ok = -1;
                    end
                end
            else
                test = oopl > 0;
                u = uu(test);
                ray = rray(test);
                opl = oopl(test);
                if ~(any(test))
                    if nargout < 4
                        error('FindForwardRayToPoint: all p behind ray.p_');
                    else
                        ok = -1;
                    end
                end
            end
        end
        
        function [u,ray, opl, ok] = FindBackwardRayToPoint(obj, p)
            [uu, rray, oopl, ok] = obj.FindRayToPoint(p);
            if ok < 0 && nargout < 4
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
                        ok = -1;
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
                        ok = -1;
                    end
                end
            end
        end

        function [u, ray, opl, ok] = FindRayToPoint(obj, p)
            arguments
                obj
                p (2,1) double
            end
            ok = 1;
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
                end % now testmin * testmax > 0
                if testmin > 0 % both positive, look for minimum
                    testfun = @(uu) obj.TestIntersect(uu, p);
                else % both negative, look for maximum
                    testfun = @(uu) - obj.TestIntersect(uu, p);
                end
                [uu, fval] = fminbnd(testfun, obj.umin_, obj.umax_,optimset('OutputFcn',@myoutput,'Display','off'));
                if fval == 0
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
                    if nargout < 4
                        error('RayBundle.FindRayToPoint: No intersection found');
                    else
                        u = uu;
                        ray = obj.Ray(u);
                        opl = (p - ray.p_)' * ray.k_;
                        ok = -1;
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
        
        function rv = PropagateTo(obj, locationRange, nameValueArgs)
            arguments
                obj
                locationRange LocationRange
                nameValueArgs.nPoints (1,1) {mustBeInteger, mustBeGreaterThan(nameValueArgs.nPoints, 1)} = 100
                nameValueArgs.errorOnCaustic (1,1) logical = true
                nameValueArgs.refractIndex (1,1) double = NaN
            end
            
            doRefract = ~(isnan(nameValueArgs.refractIndex));
            n = nameValueArgs.nPoints;
            [u0, ~, ~, ok0] = FindRayToPoint(obj, locationRange.Loc(locationRange.umin_));
            if ok0 < 0
                u0 = NaN;
            end
            [u1, ~, ~, ok1] = FindRayToPoint(obj, locationRange.Loc(locationRange.umax_));
            if ok1 < 0
                u1 = NaN;
            end
            urange = linspace(max(obj.umin_, min(u0,u1)), min(obj.umax_, max(u0, u1)), n);
            u_loc = NaN(1,n);
            p_loc = NaN(2,n);
            k_loc = NaN(2,n);
            n_loc = NaN(2,n);
            for i = 1:nameValueArgs.nPoints
                ray = obj.Ray(urange(i));
                [u_loc(i), p_loc(:,i), n_loc(:,i)] = locationRange.Intersect(ray);
                if doRefract
                    tmp = ray.RefractAtPoint(p_loc(:,i), n_loc(:,i), nameValueArgs.refractIndex);
                    k_loc(:,i) = tmp.k_;
                else
                    k_loc(:,i) = ray.k_;
                end
            end
            du_loc = diff(u_loc);
            allPos = (du_loc > 0);
            allNeg = (du_loc < 0);
            if nameValueArgs.errorOnCaustic && (~(all(allPos)|| all(allNeg)))
                error('RayBundle.PropagateTo: Encountered caustic');
            end
            rv = SplineRayBundle(p_loc, k_loc, []);
        end
        
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

        
    end
    methods (Access = private)
        function rv = TestIntersect(obj, u, p)
            ray = obj.Ray(u);
            rv = Cross2D(p - ray.p_, ray.k_);
        end
        function [u, ray, opl] = FindBracketedRayToPoint(obj, p, u0, u1)
            func = @(uu) obj.TestIntersect(uu, p);
            u = FindRoot1D(func, u0, u1);
            ray = obj.Ray(u);
            opl = (p - ray.p_)' * ray.k_;
        end
    end
    
end