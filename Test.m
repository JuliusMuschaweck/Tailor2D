clear;
%%
ll = LocationStraight([0;-1],[0;1]);
r = Ray([-2;0], [1;0.1]);
[uu, pp] = ll.Intersect(r);
ll.Normal(0.5) % [1;0]
ll.Tangent(0.5) % [0;2]
ll.SetRange(-1,1);
ll.Tangent(0)
%%
la = LocationArc([0;0],2,0,pi/2); % circle from 0 to pi/2 = 1. quadrant, radius 2
r = Ray([-2;0], [1;0.1]);
[uu, pp] = la.Intersect(r) % [0.1296, [1.9604 0.3960]
la.Normal(0.5) % sqrt(0.5)*[1;1];
la.Tangent(0.5) 
la.SetRange(0,pi/2);
la.Tangent(pi/4)
%% 
n = 10;
u1 = linspace(0,1,n);
xx = u1.^2 * 0.5;
yy = linspace(-1,1,n);
locs = [xx;yy];
dirs = locs - [-3;0];
for i = 1:n
    dirs(:,i) = Unit2D(dirs(:,i));
end
rayb = SplineRayBundle(locs, dirs, []);

figure(1);
clf;
uu = linspace(0,1,21);
iloc = rayb.Loc(uu);
ik = rayb.Dir(uu);
irays = rayb.Ray(uu);
plot(iloc(1,:),iloc(2,:),'r');
hold on;
iloc2 = iloc + 0.1*ik;
for i = 1:length(uu)
   plot([iloc(1,i),iloc2(1,i)],[iloc(2,i),iloc2(2,i)],'b');
end
axis equal;
%%
n = 36;
u1 = linspace(0,pi,n);
xx = cos(u1);
yy = sin(u1);
locs = [xx;yy];
dirs = [-yy;xx];
for i = 1:n
    dirs(:,i) = Unit2D(dirs(:,i));
end
rayb = SplineRayBundle(locs, dirs, []);

figure(2);
clf;
uu = linspace(0,1,21);
iloc = rayb.Loc(uu);
ik = rayb.Dir(uu);
irays = rayb.Ray(uu);
plot(iloc(1,:),iloc(2,:),'r');
hold on;
iloc2 = iloc + 0.1*ik;
for i = 1:length(uu)
   plot([iloc(1,i),iloc2(1,i)],[iloc(2,i),iloc2(2,i)],'b');
end
axis equal;
%%
sl = SurfaceSlope2D([0;2], Unit2D([1;1]), true)
sl = SurfaceSlope2D([1;1], [1;-1], false)
sl = SurfaceSlope2D([1;1], [1;-1], true)

