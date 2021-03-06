clear;
close all;
%%
r = 2;
alpha = linspace(-0.1,pi+0.1,10);
ca = cos(alpha);
sa = sin(alpha);
loc = r * [ca;sa];
dirLeft = [-sa; ca];
dirRight = -dirLeft;
rblLeft = SplineRayBundle(loc, dirLeft,[]);
rblRight = SplineRayBundle(loc, dirRight,[]);
startInvolute = [r+1e-3;0];
odefun = @(t, y) i_odefun(t, y, rblRight, rblLeft, [1;0]);
tspan = [0,10];
y0 = [r;0];
[t,y] = ode45(odefun, tspan, y0, odeset('Events',@eventsFcn_ym2));
testzero = [y(end,1) + 2, y(end,2) - 2 * pi];

%%
figure(1);
clf;
uu = linspace(0,1,21);
iloc = rblLeft.Loc(uu);
ik = rblLeft.Dir(uu);
irays = rblLeft.Ray(uu);
plot(iloc(1,:),iloc(2,:),'r');
hold on;
iloc2 = iloc + 0.1*ik;
for i = 1:length(uu)
   plot([iloc(1,i),iloc2(1,i)],[iloc(2,i),iloc2(2,i)],'b');
end
plot(y(:,1),y(:,2));
axis equal;
grid on;

%%



function dy = i_odefun(t, y, in_rbl, out_rbl, dy0)
    if t == 0
        dy = dy0;
        return;
    end
    [~, ray1] = in_rbl.FindForwardRayToPoint(y);
    [~, ray2] = out_rbl.FindBackwardRayToPoint(y);
    dy = SurfaceSlope2D(ray1.k_, ray2.k_, true);    
end

function [value,isterminal,direction] = eventsFcn_ym2(~,y)
    value = y(1)+2;
    isterminal = 1;
    direction = 0;
end