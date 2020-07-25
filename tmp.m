clear;
rb = SphericalWavefrontRayBundle([0;0], 0, -0.1, pi/2, 1);
s = LocationStraight([1;-1],[1;1]);
test1 = rb.PropagateTo(s);
test2 = rb.PropagateTo(s, 'nPoints',10, 'refractIndex', 1.5);

figure(1);
clf;
axis equal;
hold on;
PlotRayBundle(gca,rb,1,'r');
[px,py] = s.Points(2);
plot(px,py,'k');
PlotRayBundle(gca, test1,1,'b');
PlotRayBundle(gca, test2,1,'g');