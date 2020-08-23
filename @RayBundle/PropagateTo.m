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
    r0 = obj.Ray(obj.umin_);
    r1 = obj.Ray(obj.umax_);
    l0 = locationRange.Loc(locationRange.umin_);
    l1 = locationRange.Loc(locationRange.umax_);    
    
    % see if location edges within ray bundle or outside
    % l01inside are u values for ray bundle, NaN if outside
    [l0inside, ray_to_l0, ~, l0ok] = FindRayToPoint(obj, locationRange.Loc(locationRange.umin_));
    [l1inside, ray_to_l1, ~, l1ok] = FindRayToPoint(obj, locationRange.Loc(locationRange.umax_));
    % see if edge rays intersect location,
    % r01inside are u values for location, NaN if outside
    [r0inside, ~, ~, r0ok] = locationRange.Intersect(obj.Ray(obj.umin_));
    [r1inside, ~, ~, r1ok] = locationRange.Intersect(obj.Ray(obj.umax_));
    % see if edge rays precisely intersect location edge

    if sum([l0ok, l1ok, r0ok, r1ok]) == 0 % empty intersection
        error('RayBundle.PropagateTo: No intersection with target');
    end
    if sum([l0ok, l1ok, r0ok, r1ok]) == 1 % 
        error('RayBundle.PropagateTo: No intersection with target, sum == 1 (this cannot happen)');
    end
    % now at least two ok
    if r0ok
        umin = obj.umin_;
    else
        umin = min(l0inside, l1inside);
    end
    if r1ok
        umax = obj.umax_;
    else
        umax = max(l0inside, l1inside);
    end
    if any(isnan([umin,umax]))
        error('RayBundle.PropagateTo: No intersection with target, isnan([umin,umax]) (this cannot happen)');
    end
    urange = linspace(umin, umax, n);
    u_loc = NaN(1,n);
    p_loc = NaN(2,n);
    k_loc = NaN(2,n);
    n_loc = NaN(2,n);
    % set location end points to avoid missing ray / roundoff error
    if l0ok
        if l0inside == umin
            idx = 1;
        elseif l0inside == umax
            idx = n;
        else
            error('RayBundle.PropagateTo: this cannot happen');
        end
        uu = locationRange.umin_;
        pp = l0;
        nn = locationRange.Normal(uu);
        u_loc(idx) = uu;
        p_loc(:,idx) = pp;
        n_loc(:,idx) = nn;
        if doRefract
            tmp = ray_to_l0.RefractAtPoint(pp, nn, nameValueArgs.refractIndex);
            k_loc(:,idx) = tmp.k_;
        else
            k_loc(:,idx) = ray_to_l0.k_;
        end
    end
    if l1ok
        if l1inside == umin
            idx = 1;
        elseif l1inside == umax
            idx = n;
        else
            error('RayBundle.PropagateTo: this cannot happen');
        end
        uu = locationRange.umax_;
        pp = l1;
        nn = locationRange.Normal(uu);
        u_loc(idx) = uu;
        p_loc(:,idx) = pp;
        n_loc(:,idx) = nn;
        if doRefract
            tmp = ray_to_l1.RefractAtPoint(pp, nn, nameValueArgs.refractIndex);
            k_loc(:,idx) = tmp.k_;
        else
            k_loc(:,idx) = ray_to_l1.k_;
        end
    end
    if r0ok
        idx = 1;
        uu = r0inside;
        pp = locationRange.Loc(uu);
        nn = locationRange.Normal(uu);
        u_loc(idx) = uu;
        p_loc(:,idx) = pp;
        n_loc(:,idx) = nn;
        if doRefract
            tmp = r0.RefractAtPoint(pp, nn, nameValueArgs.refractIndex);
            k_loc(:,idx) = tmp.k_;
        else
            k_loc(:,idx) = r0.k_;
        end
    end
    if r1ok
        idx = n;
        uu = r1inside;
        pp = locationRange.Loc(uu);
        nn = locationRange.Normal(uu);
        u_loc(idx) = uu;
        p_loc(:,idx) = pp;
        n_loc(:,idx) = nn;
        if doRefract
            tmp = r1.RefractAtPoint(pp, nn, nameValueArgs.refractIndex);
            k_loc(:,idx) = tmp.k_;
        else
            k_loc(:,idx) = r1.k_;
        end
    end

    if isnan(u_loc(1))
        i0 = 1;
    else
        i0 = 2;
    end
    if isnan(u_loc(end))
        i1 = n;
    else
        i1 = n-1;
    end
    for i = i0:i1
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

