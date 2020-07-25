function [rv, sol] = TailorEdgeRaySurface( rayBundleIn, rayBundleOut, nameValueArgs)
    % ray bundle in, ray bundle out.
    % start point: can be defined by 
    %   1. a 2D point: Then we find the in and out edge rays which meet there
    %   2. u values for in and out bundle. They define two rays and the start point is where they meet
    %       default: umin for both in and out
    % start direction: 'left' or 'right' as seen from starting in ray
    %   when using default umin: 'auto': in.Ray(in.umin_ + eps) on left or right side of start point
    % end condition: 
    %   1. default: running out of either in or out rays
    %   2. intersecting a boundary line, a LocationRange
    arguments
        rayBundleIn (1,1) RayBundle
        rayBundleOut (1,1) RayBundle
        nameValueArgs.start_mode {mustBeStartMode} = 'u' % or 'point'
        nameValueArgs.start_point (2,1) double = NaN(2,1)
        nameValueArgs.start_uIn (1,1) double = rayBundleIn.umin_
        nameValueArgs.start_uOut (1,1) double = rayBundleOut.umin_
        nameValueArgs.start_dir {mustBeStartDir} = 'auto'
        nameValueArgs.end_mode {mustBeEndMode} = 'default'
        nameValueArgs.end_location (1,1) LocationRange = LocationStraight(NaN(2,1), NaN(2,1))
        nameValueArgs.nPoints (1,1) {mustBeInteger, mustBeGreaterThan(nameValueArgs.nPoints,1)} = 50
    end
    % determine starting point
    if strcmp(nameValueArgs.start_mode, 'u') % given u values for in and out
        r_in_0 = rayBundleIn.Ray(nameValueArgs.start_uIn);
        r_out_0 = rayBundleOut.Ray(nameValueArgs.start_uOut);
        %u_in_0 = nameValueArgs.start_uIn;
        %u_out_0 = nameValueArgs.start_uOut;
        p0 = r1.Intersect(r2);
        if isnan(p0)
            error('TailorEdgeRaySurface: start rays are parallel, no start point found');
        end
    else % 'point', see which rays meet there
        p0 = nameValueArgs.start_point;
        if isnan(p0)
            error('TailorEdgeRaySurface: Use (...''start_point'', p,...) to define start point');
        end
        [~, r_in_0] = rayBundleIn.FindForwardRayToPoint(p0);
        [~, r_out_0] = rayBundleOut.FindBackwardRayToPoint(p0);
    end
    % determine surface direction, left/right
    left = true;
    right = false;
    if strcmp(nameValueArgs.start_dir,'left')
        start_dir = left;
    elseif strcmp(nameValueArgs.start_dir,'right')
        start_dir = right;
    else % 'auto'
        error('TailorEdgeRaySurface: ''start_dir'',''auto'' not imiplemented');
    end
    % determine starting slope
    slope0 = SurfaceSlope2D(r_in_0.k_, r_out_0.k_, start_dir);
    % create ODE function
    odefun = @(t, y) i_odefun(t, y, rayBundleIn, rayBundleOut, slope0, start_dir);
    solver = RayTrace2D_ODEfun();
    % solve
    tspan = [0,Inf];
    sol = solver.Solve(odefun, tspan, p0, odeset('RelTol',1e-6));
    p = deval(sol,linspace(sol.x(1), sol.x(end), nameValueArgs.nPoints));
    rv = LocationSpline(p);
end

function mustBeStartMode(s)
    if strcmp(s, 'u') || strcmp(s,'point')
        return;
    end
    error('TailorEdgeRaySurface: start_mode must be ''u'' or ''point''');
end
function mustBeStartDir(s)
    if strcmp(s, 'auto') || strcmp(s,'left') || strcmp(s,'right')
        return;
    end
    error('TailorEdgeRaySurface: start_dir must be ''auto'' or ''left'' or ''right''');
end
function mustBeEndMode(s)
    if strcmp(s, 'default') || strcmp(s,'intersect') 
        return;
    end
    error('TailorEdgeRaySurface: end_mode must be ''default'' or ''intersect''');
end

function [dy, ok] = i_odefun(t, y, in_rbl, out_rbl, dy0, left)
    if t == 0
        dy = dy0;
        ok = true;
        return;
    end
    [~, ray1, ~, ok_in] = in_rbl.FindForwardRayToPoint(y);
    [~, ray2, ~, ok_out] = out_rbl.FindBackwardRayToPoint(y);
    if ok_in < 0 || ok_out < 0
        dy = [0;0];
        ok = false;
    else
        dy = SurfaceSlope2D(ray1.k_, ray2.k_, left);
        ok = true;
    end
end

