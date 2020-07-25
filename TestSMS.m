clear;
%% set up system
s_y = 1; % sources at [0; +- s_y]
t_x  = 10;
t_y = 1.5; % targets at [t_x; +- t_y]
xe = [4;0]; % center point on first lens surface
ct = 2; % center thickness
n_air = 1;
n_glass = 1.5;
Src1 = SphericalWavefrontRayBundle([0;s_y], 0, -pi/2, pi/2, n_air);
Src2 = SphericalWavefrontRayBundle([0;-s_y], 0, -pi/2, pi/2, n_air);
Tgt1 = SphericalWavefrontRayBundle([t_x;-t_y], 0, -pi/2, -3 * pi/2, n_air);
Tgt2 = SphericalWavefrontRayBundle([t_x;t_y], 0, -pi/2, -3 * pi/2, n_air);

% rays from both sources to center of first surface, then refracted into glass
r1a = Ray(Src1.c_, Unit2D( xe - Src1.c_));
r1b = r1a.RefractAtPoint(xe,[1;0],n_glass);
r2a = Ray(Src2.c_, Unit2D( xe - Src2.c_));
r2b = r2a.RefractAtPoint(xe,[1;0],n_glass);
% first two points on second surface and the rays to targets from there
p1 = r1b.p_ + ct * Unit2D(r1b.k_);
p2 = r2b.p_ + ct * Unit2D(r2b.k_);
r1c = Ray(p1, n_air * Unit2D(Tgt1.c_ - p1));
r2c = Ray(p2, n_air * Unit2D(Tgt2.c_ - p2));
% and their normals
n1 = Unit2D(r1b.k_ - r1c.k_);
n2 = Unit2D(r2b.k_ - r2c.k_);

% circular center segment of surface 2: circle center where normals meet
test1 = Ray(p1,n1);
test2 = Ray(p2,n2);
cc = test1.IntersectRay(test2);
rr = p1-cc;
so1 = LocationArc(cc, Norm2D(rr), atan2(rr(2), rr(1)), -2*pi + atan2(-rr(2), rr(1)));

% tailor first segment of first surface

%% plot
figure(1);
clf;
hold on;
% plot initial setup points
scatter(Src1.c_(1), Src1.c_(2), 'ro');
scatter(Src2.c_(1), Src2.c_(2), 'ro');
scatter(Tgt1.c_(1), Tgt1.c_(2), 'bo');
scatter(Tgt2.c_(1), Tgt2.c_(2), 'bo');
scatter(xe(1), xe(2), 'kx');
scatter(p1(1), p1(2), 'kx');
scatter(p2(1), p2(2), 'kx');
% plot first two ray paths
PlotRay(r1a, Norm2D(xe - Src1.c_),'g');
PlotRay(r1b, Norm2D(p1 - xe),'g');
PlotRay(r1c, Norm2D(Tgt1.c_ - p1),'g');
PlotRay(r2a, Norm2D(xe - Src2.c_),'m');
PlotRay(r2b, Norm2D(p2 - xe),'m');
PlotRay(r2c, Norm2D(Tgt2.c_ - p2),'m');
% surface normals at first two points
PlotSurfaceNormal (p1, n1);
PlotSurfaceNormal (p2, n2);
% initial circle segment
scatter(cc(1), cc(2), 'kd');
PlotLocation(so1, 10);


axis equal;

function PlotRay( ray, length, varargin )
    p2 = ray.p_ + length * Unit2D(ray.k_);
    plot( gca, [ray.p_(1), p2(1)], [ray.p_(2), p2(2)], varargin{:});
end

function PlotSurfaceNormal( p, n )
    tmp = p + n;
    plot(gca, [p(1), tmp(1)], [p(2), tmp(2)], 'k-.');
end

function PlotLocation(loc, n)
    [px, py] = loc.Points(n);
    plot(gca, px, py, 'k');
end