clear;
figure(1);
clf;
hold on;
axis equal;
grid on;
%% set up system
s_y = 0.5; % sources at [0; +- s_y]
t_x  = 10;
t_y = 0.6; % targets at [t_x; +- t_y]
xe = [3.80;0]; % center point on first lens surface
ct = 1.5; % center thickness
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
% plot surface normals at first two points
PlotSurfaceNormal (p1, n1);
PlotSurfaceNormal (p2, n2);

% circular center segment of surface 2: circle center where normals meet
test1 = Ray(p1,n1);
test2 = Ray(p2,n2);
cc = test1.IntersectRay(test2);
rr = p1-cc;
if cc(1) > p1(1) % circle center to the right, atan2 -> [-pi;pi]
    theta_0 = atan2(rr(2), rr(1));
    theta_1 = -2*pi + atan2(-rr(2), rr(1));
else
    theta_0 = atan2(-rr(2), rr(1));
    theta_1 = atan2(rr(2), rr(1));
end
sOC = LocationArc(cc, Norm2D(rr), theta_0, theta_1);
% plot initial circle segment
scatter(cc(1), cc(2), 'kd');
PlotLocation(sOC, 10);

%% tailor first segment of first surface
i = 1;
rbO{i} = Tgt1.PropagateTo(sOC,'RefractIndex',n_glass);
PlotRayBundle(gca,rbO{i}.Reverse(),0.2,'g');
sE{i} = TailorEdgeRaySurface(Src1,ProlongedRayBundle_Collimated(rbO{i},0.1,0),'start_mode','point','start_point',xe,'start_dir','left');
PlotLocation(sE{i}, 10);
drawnow;
%% tailor first segment of second surface
rbE{i} = Src2.PropagateTo(sE{i},'RefractIndex',n_glass);
PlotRayBundle(gca,rbE{i},0.2,'m');
sO{i} = TailorEdgeRaySurface(ProlongedRayBundle_Collimated(rbE{i},0.1,0),Tgt2,'start_mode','point','start_point',sOC.Loc(sOC.umin_),'start_dir','left');
PlotLocation(sO{i}, 10);
drawnow;
%% tailor next segment of first surface
for i = 2:10
    rbO{i} = Tgt1.PropagateTo(sO{i-1},'RefractIndex',n_glass);
    PlotRayBundle(gca,rbO{i}.Reverse(),0.2,'g');
    sE{i} = TailorEdgeRaySurface(Src1,ProlongedRayBundle_Collimated(rbO{i},0.1,0),'start_mode','point','start_point',sE{i-1}.Loc(sE{i-1}.umax_),'start_dir','left');
    PlotLocation(sE{i}, 10);
    drawnow;
    % tailor next segment of second surface
    rbE{i} = Src2.PropagateTo(sE{i},'RefractIndex',n_glass);
    PlotRayBundle(gca,rbE{i},0.2,'m');
    sO{i} = TailorEdgeRaySurface(ProlongedRayBundle_Collimated(rbE{i},0.1,0),Tgt2,'start_mode','point','start_point',sO{i-1}.Loc(sO{i-1}.umax_),'start_dir','left');
    PlotLocation(sO{i}, 10);
    drawnow;
end

%% prepare export 
np = 10; 
EntrancePoints = zeros(2, np * i + 1);
EntrancePoints(:,1) = xe;
col = 1;
for i = 1:10
    surf = sE{i};
    for j = 1:np
        u = surf.umin_ * (np-j)/np + surf.umax_ * j / np;
        col = col + 1;
        EntrancePoints(:,col) = surf.Loc(u);
    end
end

ExitPoints = zeros(2, np * i + np/2 + 1);
col = 0;
for j = (np/2) : np
    u = sOC.umin_ * j/np + sOC.umax_ * (np-j) / np;
    col = col + 1;
    ExitPoints(:,col) =sOC.Loc(u);
end
for i = 1:10
    surf = sO{i};
    for j = 1:np
        u = surf.umin_ * (np-j)/np + surf.umax_ * j / np;
        col = col + 1;
        ExitPoints(:,col) = surf.Loc(u);
    end
end

plot(EntrancePoints(1,:), EntrancePoints(2,:),'r','LineWidth',1);
plot(ExitPoints(1,:), ExitPoints(2,:),'b','LineWidth',1);

LT_Entrance = fliplr((EntrancePoints - [xe(1);0])');
LT_Entrance = LT_Entrance * [1,0;0,-1];
LT_Exit = fliplr((ExitPoints - [ExitPoints(1,1);0])');
Actual_CT = ExitPoints(1,1) -EntrancePoints(1,1);
% set LT center thickness to 1


%% plot helper functions
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