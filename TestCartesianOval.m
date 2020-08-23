clear;
figure(2);
clf;
deg = pi/180;
n_air = 1.0;
n_glass = 1.5;
Src = SphericalWavefrontRayBundle([0;0], 0, -30*deg, 30*deg, n_air);
Tgt = SphericalWavefrontRayBundle([10,0], 0, -150*deg, -210*deg, n_glass);

p0 = [5;0];
surf = TailorEdgeRaySurface(Src,Tgt,'start_mode','point','start_point',p0,'start_dir','left');

[px, py] = surf.Points(100);
plot(px, py, 'k');
hold on;
grid on;
axis equal;
scatter([0,10],[0,0],'k');

LTPts = [py',-px'+5];
% In LightTools, the rays go to 10,0 within a few nanometers -> ok
%%
refracted = Src.PropagateTo(surf,'RefractIndex',n_glass);
PlotRayBundle(gca, refracted,5,'m');

%%
rr = Ray([0;0],[1,0]);
r2 = rr.RefractAtPoint([5;0],Unit2D([1;1]),sqrt(2));
testzero = sin(atan(r2.k_(2)/r2.k_(1))) - 15*pi/180
