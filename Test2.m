clear;
xx = linspace(0,1,11);
y1 = sin(xx);
y2 = cos(xx);
y3 = xx.^2;
yy = cat(1,y1,y2,y3);
pp = spline(xx, yy);

test = ppval(pp,xx);
dpp = pp;
%dpp.coefs = dpp.coefs * [0 3 0 0; 0 0 2 0; 0 0 0 1; 0 0 0 0];
dpp.coefs = dpp.coefs * [3 0 0; 0 2 0; 0 0 1; 0 0 0];
dpp.order = 3;
dtest = ppval(dpp,xx);

