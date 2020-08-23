clear;
ni = sqrt(2)-1e-15;
no = 1;
n = [1;0];
in = ni * Unit2D([1;1]);
out = in + (sqrt(no^2 - ni^2 + (in' * n)^2) - in' * n) * n
a_in = Cross2D(Unit2D(in),n);
a_out = Cross2D(Unit2D(out),n);
a_in * ni - a_out * no

ri = Ray([0;1], ni * Unit2D([1;-1]));
p = [1;0];
ro = ri.RefractAtPoint(p, n, no)