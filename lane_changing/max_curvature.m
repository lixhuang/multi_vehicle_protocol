syms x y d2;
x0 = -10;
y0 = 0;
d1 = 2;
beta = 6;
a = 2;
b = 1;
fox = beta/b*(y-y0)^(beta-1);
foy = -beta/a*(x-x0)^(beta-1);
foxn = fox/(fox^2+foy^2);
foyn = foy/(fox^2+foy^2);
fgxn = -x/(x^2+y^2);
fgyn = -y/(x^2+y^2);
A=[1 d2 d2^2 d2^3; 0 1 2*d2 3d2^2; 1 d1 d1^2 d1^3; 0 1 2*d1 3d1^2];
sig_vec = [1; 0; 0; 0];
a_vec = A^(-1)*sig_vec;
d = ((x-x0/a))^beta+((y-y0/a))^beta;
sigma = [1 d d^2 d^3]*a_vec;
Fx = (1-sigma)*foxn+sigma*fgxn;
Fy = (1-sigma)*foyn+sigma*fgyn;
M = [diff(Fx,x) diff(Fx,y); diff(Fy,x) diff(Fy,y)];
M_func = matlabFunction(M);
%curv = det(M);

len = length(theta);
curve_log = [];
d_max = 0;
for temp_d = d1:0.1:4
    [r, theta] = meshgrid([sqrt(d1):0.1:sqrt(temp_d)],[0:0.01:2*pi]);
    px = r.*cos(theta)*a+x0;
    py = r.*cos(theta)*b+y0;
    sz = size(px);
    max = 0;
    x_max= 0;
    y_max= 0;
    for i = sz(1)
        for j = 1:sz(2)
            M_max = M_func(temp_d, px(i,j), py(i,j));
            c_max = abs(det(M_max));
            if(c_max>max)
                max = c_max;
                x_max= px(i,j);
                y_max= py(i,j);
            end
        end
    end
    curve_log = [curve_log;max];
end

