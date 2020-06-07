clear;
close all;
%% geometry setup
r = 0.6; % radius of tubes
D = 12; % distance of tube centers
involuteEndAngle = acos(r / (D/2)); % top left involute end edge ray is tangent to left circle here

left = true;
right = false;
%% upper half = left involute + top quasi ellipse + right involute
% starting with left involute
rblIn1 = CircularArcEdgeRayBundle([0;0], r, involuteEndAngle, pi);
rblOut1 = CircularArcEdgeRayBundle([0;0], r, pi, involuteEndAngle);
startInvolute = [-r;0];
slope0 = [-1;0];
odefun = @(t, y) i_odefun(t, y, rblIn1, rblOut1, slope0, right);
tspan = [0,Inf];
solver = RayTrace2D_ODEfun();
tic;
sol1 = solver.Solve(odefun, tspan, startInvolute, odeset('RelTol',1e-6));
toc
y_ltinv = deval(sol1,linspace(sol1.x(1), sol1.x(end), 50));
%% continue with top reflector
rblIn2 = CircularArcEdgeRayBundle([0;0], r, -involuteEndAngle, involuteEndAngle);
rblOut2 = CircularArcEdgeRayBundle([D;0], r, pi - involuteEndAngle, pi + involuteEndAngle);
startReflTop = sol1.y(:,end);
slope0 = SurfaceSlope2D(rblIn2.Ray(rblIn2.umax_).k_, rblOut2.Ray(rblOut2.umax_).k_, right);
odefun2 = @(t, y) i_odefun(t, y, rblIn2, rblOut2, slope0, right);
tic;
sol2 = solver.Solve(odefun2, tspan+1, startReflTop, odeset('RelTol',1e-6));
toc
y_trefl = deval(sol2,linspace(sol2.x(1), sol2.x(end), 50));
%% and with top right involute
rblIn3 = CircularArcEdgeRayBundle([D;0], r, pi-involuteEndAngle, 0);
rblOut3 = CircularArcEdgeRayBundle([D;0], r, 0, pi - involuteEndAngle);
startReflTop = sol2.y(:,end);
slope0 = SurfaceSlope2D(rblIn3.Ray(rblIn3.umin_).k_, rblOut3.Ray(rblOut3.umax_).k_, right);
odefun3 = @(t, y) i_odefun(t, y, rblIn3, rblOut3, slope0, right);
tic
sol3 = solver.Solve(odefun3, tspan, startReflTop, odeset('RelTol',1e-6));
toc;
y_rtinv = deval(sol3,linspace(sol3.x(1), sol3.x(end), 50));

%% same for bottom
% starting with left involute
rblIn1b = CircularArcEdgeRayBundle([0;0], r, -involuteEndAngle, -pi);
rblOut1b = CircularArcEdgeRayBundle([0;0], r, -pi, -involuteEndAngle);
startInvolute = [-r;0];
slope0 = [-1;0];
odefun = @(t, y) i_odefun(t, y, rblIn1b, rblOut1b, slope0, left);
tspan = [0,Inf];
solver = RayTrace2D_ODEfun();
sol1b = solver.Solve(odefun, tspan, startInvolute, odeset('RelTol',1e-6));
y_lbinv = deval(sol1b,linspace(sol1b.x(1), sol1b.x(end), 50));
%% continue with bottom reflector
rblIn2b = CircularArcEdgeRayBundle([0;0], r, involuteEndAngle, -involuteEndAngle);
rblOut2b = CircularArcEdgeRayBundle([D;0], r, -pi + involuteEndAngle, -pi - involuteEndAngle);
startReflTop = sol1b.y(:,end);
slope0 = SurfaceSlope2D(rblIn2b.Ray(rblIn2b.umax_).k_, rblOut2b.Ray(rblOut2b.umax_).k_, left);
odefun2 = @(t, y) i_odefun(t, y, rblIn2b, rblOut2b, slope0, left);
sol2b = solver.Solve(odefun2, tspan+1, startReflTop, odeset('RelTol',1e-6));
y_brefl = deval(sol2b,linspace(sol2b.x(1), sol2b.x(end), 50));
%% and with bottom right involute
rblIn3b = CircularArcEdgeRayBundle([D;0], r, - pi+involuteEndAngle, 0);
rblOut3b = CircularArcEdgeRayBundle([D;0], r, 0, -pi + involuteEndAngle);
startReflTop = sol2b.y(:,end);
slope0 = SurfaceSlope2D(rblIn3b.Ray(rblIn3b.umin_).k_, rblOut3b.Ray(rblOut3.umax_).k_, left);
odefun3 = @(t, y) i_odefun(t, y, rblIn3b, rblOut3b, slope0, left);
sol3b = solver.Solve(odefun3, tspan, startReflTop, odeset('RelTol',1e-6));
y_rbinv = deval(sol3b,linspace(sol3b.x(1), sol3b.x(end), 50));

%%
f1 = figure(1);
clf;
axis equal;
%grid on;
hold on;
PlotRayBundle(gca, rblIn1, 0);
plot(y_ltinv(1,:),y_ltinv(2,:),'r');
% top reflector
PlotRayBundle(gca, rblIn2, 0);
PlotRayBundle(gca, rblOut2, 0);
plot(y_trefl(1,:),y_trefl(2,:),'b');
% top right involute
PlotRayBundle(gca, rblIn3, 0);
plot(y_rtinv(1,:),y_rtinv(2,:),'r');

PlotRayBundle(gca, rblIn1b, 0);
plot(y_lbinv(1,:),y_lbinv(2,:),'r');
% top reflector
PlotRayBundle(gca, rblIn2b, 0);
PlotRayBundle(gca, rblOut2b, 0);
plot(y_brefl(1,:),y_brefl(2,:),'b');
% top right involute
PlotRayBundle(gca, rblIn3b, 0);
plot(y_rbinv(1,:),y_rbinv(2,:),'r');
%axis off


%%



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

function [value,isterminal,direction] = eventsFcn_ym2(~,y)
    value = y(1)+2;
    isterminal = 1;
    direction = 0;
end

function PlotRayBundle( ax, rb, kLength)
    uu = linspace(rb.umin_,rb.umax_,21);
    iloc = rb.Loc(uu);
    ik = rb.Dir(uu);
    plot(ax, iloc(1,:),iloc(2,:),'k');
    hold on;
    iloc2 = iloc + kLength*ik;
    for i = 1:length(uu)
        plot([iloc(1,i),iloc2(1,i)],[iloc(2,i),iloc2(2,i)],'b');
    end
end