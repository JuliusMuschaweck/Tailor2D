clear;
close all;
%% geometry setup
tic;
r = 2; % radius of tubes
D = 12; % distance of tube centers
involuteEndAngle = acos(r / (D/2)); % top left involute end edge ray is tangent to left circle here

%% upper half = left involute + top quasi ellipse + right involute
% starting with left involute
rblIn1 = CircularArcEdgeRayBundle([0;0], r, involuteEndAngle, pi);
rblOut1 = CircularArcEdgeRayBundle([0;0], r, pi, involuteEndAngle);
startInvolute = [-r;0];
y_ltinv = TailorEdgeRaySurface(rblIn1, rblOut1, 'start_mode', 'point', 'start_point', startInvolute, 'start_dir', 'right');
%% continue with top reflector
rblIn2 = CircularArcEdgeRayBundle([0;0], r, -involuteEndAngle, involuteEndAngle);
rblOut2 = CircularArcEdgeRayBundle([D;0], r, pi - involuteEndAngle, pi + involuteEndAngle);
startReflTop = y_ltinv.Loc(y_ltinv.umax_);
y_trefl = TailorEdgeRaySurface(rblIn2, rblOut2, 'start_mode', 'point', 'start_point', startReflTop, 'start_dir', 'right');
%% and with top right involute
rblIn3 = CircularArcEdgeRayBundle([D;0], r, pi-involuteEndAngle, 0);
rblOut3 = CircularArcEdgeRayBundle([D;0], r, 0, pi - involuteEndAngle);
startReflTop = y_trefl.Loc(y_trefl.umax_);
y_rtinv = TailorEdgeRaySurface(rblIn3, rblOut3, 'start_mode', 'point', 'start_point', startReflTop, 'start_dir', 'right');
%% same for bottom
% starting with left involute
rblIn1b = CircularArcEdgeRayBundle([0;0], r, -involuteEndAngle, -pi);
rblOut1b = CircularArcEdgeRayBundle([0;0], r, -pi, -involuteEndAngle);
startInvolute = [-r;0];
y_lbinv = TailorEdgeRaySurface(rblIn1b, rblOut1b, 'start_mode', 'point', 'start_point', startInvolute, 'start_dir', 'left');
%% continue with bottom reflector
rblIn2b = CircularArcEdgeRayBundle([0;0], r, involuteEndAngle, -involuteEndAngle);
rblOut2b = CircularArcEdgeRayBundle([D;0], r, -pi + involuteEndAngle, -pi - involuteEndAngle);
startReflTop = y_lbinv.Loc(y_lbinv.umax_);
y_brefl = TailorEdgeRaySurface(rblIn2b, rblOut2b, 'start_mode', 'point', 'start_point', startReflTop, 'start_dir', 'left');
%% and with bottom right involute
rblIn3b = CircularArcEdgeRayBundle([D;0], r, - pi+involuteEndAngle, 0);
rblOut3b = CircularArcEdgeRayBundle([D;0], r, 0, -pi + involuteEndAngle);
startReflTop = y_brefl.Loc(y_brefl.umax_);
y_rbinv = TailorEdgeRaySurface(rblIn3b, rblOut3b, 'start_mode', 'point', 'start_point', startReflTop, 'start_dir', 'left');
toc
%%
f1 = figure(1);
clf;
lw = 1.5;
nPts = 50;
axis equal;
%grid on;
hold on;
PlotRayBundle(gca, rblIn1, 0);
[px,py] = y_ltinv.Points(nPts);
plot(px,py,'r','LineWidth',lw);
% top reflector
PlotRayBundle(gca, rblIn2, 0);
PlotRayBundle(gca, rblOut2, 0);
[px,py] = y_trefl.Points(nPts);
plot(px,py,'b','LineWidth',lw);
% top right involute
PlotRayBundle(gca, rblIn3, 0);
[px,py] = y_rtinv.Points(nPts);
plot(px,py,'r','LineWidth',lw);

PlotRayBundle(gca, rblIn1b, 0);
[px,py] = y_lbinv.Points(nPts);
plot(px,py,'r','LineWidth',lw);
% top reflector
PlotRayBundle(gca, rblIn2b, 0);
PlotRayBundle(gca, rblOut2b, 0);
[px,py] = y_brefl.Points(nPts);
plot(px,py,'b','LineWidth',lw);
% top right involute
PlotRayBundle(gca, rblIn3b, 0);
[px,py] = y_rbinv.Points(nPts);
plot(px,py,'r','LineWidth',lw);

% plot some rays
r1 = rblIn1.Ray(0.5);
[~,rp1] = y_ltinv.Intersect(r1);
plot([r1.p_(1),rp1(1)],[r1.p_(2),rp1(2)],':r','LineWidth',lw);
r2 = rblIn1.Ray(0);
[~,rp2] = y_ltinv.Intersect(r2);
plot([r2.p_(1),rp2(1)],[r2.p_(2),rp2(2)],':r','LineWidth',lw);
r2a = rblOut2.Ray(1);
plot([r2.p_(1),r2a.p_(1)],[r2.p_(2),r2a.p_(2)],':r','LineWidth',lw);


r3_in = rblIn2.Ray(0.5);
[~,rp3_in] = y_trefl.Intersect(r3_in);
plot([r3_in.p_(1),rp3_in(1)],[r3_in.p_(2),rp3_in(2)],':b','LineWidth',lw);
[~,r3_out] = rblOut2.FindRayToPoint(rp3_in);
plot([r3_out.p_(1),rp3_in(1)],[r3_out.p_(2),rp3_in(2)],':b','LineWidth',lw);
r4_in = rblIn2.Ray(0.02);
[~,rp4_in] = y_trefl.Intersect(r4_in);
plot([r4_in.p_(1),rp4_in(1)],[r4_in.p_(2),rp4_in(2)],':b','LineWidth',lw);
[~,r4_out] = rblOut2.FindRayToPoint(rp4_in);
plot([r4_out.p_(1),rp4_in(1)],[r4_out.p_(2),rp4_in(2)],':b','LineWidth',lw);


axis off


%%


