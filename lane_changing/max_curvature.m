syms x y;
syms x0 y0;

beta = 1.2;
a = 4/2*2^(1/beta/2);
b = 1.8/2*2^(1/beta/2);
d2_d = 3.7+1.8/2;
d2 = (d2_d/b)^(2*beta)-1;
d1 = 0.05*d2;

fox = (y-y0)/b*(((y-y0)/b)^2)^(beta-1);
foy = -(x-x0)/a*(((x-x0)/a)^2)^(beta-1);
foxn = fox/sqrt(fox^2+foy^2);
foyn = foy/sqrt(fox^2+foy^2);
fgxn = -x/sqrt(x^2+y^2);
fgyn = -y/sqrt(x^2+y^2);
A=[1 d1 d1^2 d1^3; 0 1 2*d1 3d1^2; 1 d2 d2^2 d2^3; 0 1 2*d2 3d2^2;];
sig_vec = [1; 0; 0; 0];
a_vec = A^(-1)*sig_vec;
d = (((x-x0)/a)^2)^beta+(((y-y0)/b)^2)^beta-1;
sigma = [1 d d^2 d^3]*a_vec;
Fx = sigma*foxn+(1-sigma)*fgxn;
Fy = sigma*foyn+(1-sigma)*fgyn;

M = [jacobian(Fx,x) jacobian(Fx,y); jacobian(Fy,x) jacobian(Fy,y)]./sqrt(Fx^2+Fy^2);
M2 = [jacobian(Fx,x0) jacobian(Fx,y0); jacobian(Fy,x0) jacobian(Fy,y0)]./sqrt(Fx^2+Fy^2);
Ad = M2(2,1)*Fx./sqrt(Fx^2+Fy^2)-M2(1,1)*Fy./sqrt(Fx^2+Fy^2);
T_norm = [Fy -Fx]./sqrt(Fx^2+Fy^2)*M*[Fx;Fy]./sqrt(Fx^2+Fy^2);
T2_norm = [Fy -Fx]./sqrt(Fx^2+Fy^2)*M2*[1;0];
% Fxdx = jacobian(Fx,x);
% Fxdy = jacobian(Fx,y);
% Fydx = jacobian(Fy,x);
% Fydy = jacobian(Fy,y);
M_func = matlabFunction(M);
M2_func = matlabFunction(M2);
T_norm_func = matlabFunction(T_norm);
T2_norm_func = matlabFunction(T2_norm);
Ad_func = matlabFunction(Ad);
% Fxdx_func = matlabFunction(Fxdx);
% Fxdy_func = matlabFunction(Fxdy);
% Fydx_func = matlabFunction(Fydx);
% Fydy_func = matlabFunction(Fydy);
%curv = det(M);

curve_log = [];
d_max = 0;

M_total_max = 0;
M2_total_max = 0;
x_total_max = 0;
y_total_max = 0;
x0_total_max = 0;
y0_total_max = 0;
x2_total_max = 0;
y2_total_max = 0;
x02_total_max = 0;
y02_total_max = 0;
x0_data = -20;
y0_data = 0;

for x0_data = -40:0.5:-20
    for y0_data = -3.7/2:0.2:3.7/2
        
        r0 = (d2+1)^(1/(2*beta));
        %theta_0 = pi-asin(((-(3.7/2-1.8)-y0_data)/b/r0)^(1/beta));
        theta_0 = pi;
        [r, theta] = meshgrid([(d1+1)^(1/(2*beta)):0.1:(d2+1)^(1/(2*beta))],[pi/2:0.01:theta_0]);
        px = r.*sign(cos(theta)).*abs(cos(theta)).^(1/beta)*a+x0_data;
        py = r.*sign(sin(theta)).*abs(sin(theta)).^(1/beta)*b+y0_data;
        sz = size(px);
        max = 0;
        max2 = 0;
        x_max= 0;
        y_max= 0;
        x2_max= 0;
        y2_max= 0;
        for i = 1:sz(1)
            for j = 1:sz(2)
%                 M_max = M_func(px(i,j), x0_data, py(i,j), y0_data);
%                 M2_max = M2_func(px(i,j), x0_data, py(i,j), y0_data);
%                 [~,c_max,~] = svd(M_max);
%                 c_max = c_max(1,1);
%                 %jac(i,j)=c_max;
%                 [~,c2_max,~] = svd(M2_max);
%                 c2_max = c2_max(1,1);
%                 %jac2(i,j)=c2_max;
%                 if(c_max>max)
%                     max = c_max;
%                     x_max= px(i,j);
%                     y_max= py(i,j);
%                 end
%                 if(c2_max>max2)
%                     max2 = c2_max;
%                     x2_max= px(i,j);
%                     y2_max= py(i,j);
%                 end

                T_max = abs(T_norm_func(px(i,j), x0_data, py(i,j), y0_data));
                if(T_max>max)
                max = T_max;
                x_max= px(i,j);
                y_max= py(i,j);
                end
% 
%                 T_max = T2_norm_func(px(i,j), x0_data, py(i,j), y0_data);
%                 %T_max = sqrt(sum(T.^2));
%                 if(T_max>max)
%                 max = T_max;
%                 x_max= px(i,j);
%                 y_max= py(i,j);
%                 end
                  
            end
        end
        if(max>M_total_max)
            M_total_max = max;
            x_total_max = x_max;
            y_total_max = y_max;
            x0_total_max = x0_data;
            y0_total_max = y0_data;
        end
         if(max2>M2_total_max)
            M2_total_max = max;
            x2_total_max = x_max;
            y2_total_max = y_max;
            x02_total_max = x0_data;
            y02_total_max = y0_data;
        end
    end
end





fh1 = @(x,y,x0,y0)(((x-x0)/a)^2)^beta+(((y-y0)/b)^2)^beta-1-d1;
fh2 = @(x,y,x0,y0)(((x-x0)/a)^2)^beta+(((y-y0)/b)^2)^beta-1-d2;








%mesh(px,py,jac2);








b = 0.5;
b2 = 1;
b3 = 0.5;
%nlcon1 = @(x)-(((x(1)-x0)/a)^beta+((x(2)-y0)/b)^beta-1-d1);
%nlcon2 = @(x)(((x(1)-x0)/a)^beta+((x(2)-y0)/b)^beta-1-d2);
%nlcon1 = @(x)-fh1(x(1),x(2),x(3),x(4));
%nlcon2 = @(x)fh2(x(1),x(2),x(3),x(4));
nlcon1 = @(x)-fh1(x(1),x(2),x0_total_max,y0_total_max);
nlcon2 = @(x)fh2(x(1),x(2),x0_total_max,y0_total_max);

% [real_pos_M,real_max_M] = fmincon(@(x)-fun2(x(1:2),M_func,x(3),x(4)), .....
%     [x_total_max;y_total_max;x0_total_max;y0_total_max], [],[],[],[],.....
%     [x_total_max-b;y_total_max-b;x0_total_max-b2;y0_total_max-b3],.....
%     [x_total_max+b;y_total_max+b;x0_total_max+b2;y0_total_max+b3], @(x)nlconn(x,nlcon1,nlcon2));
% 
% [real_pos_M2,real_max_M2] = fmincon(@(x)-fun2(x(1:2),M2_func,x(3),x(4)), .....
%     [x_total_max;y_total_max;x0_total_max;y0_total_max], [],[],[],[],.....
%     [x_total_max-b;y_total_max-b;x0_total_max-b2;y0_total_max-b3],.....
%     [x_total_max+b;y_total_max+b;x0_total_max+b2;y0_total_max+b3], @(x)nlconn(x,nlcon1,nlcon2));

% [real_pos_M2,real_max_M2] = fmincon(@(x)-fun3(x(1:2),T_norm_func,x(3),x(4)), .....
%     [x_total_max;y_total_max;x0_total_max;y0_total_max], [],[],[],[],.....
%     [x_total_max-b;y_total_max-b;x0_total_max-b2;y0_total_max-b3],.....
%     [x_total_max+b;y_total_max+b;x0_total_max+b2;y0_total_max+b3], @(x)nlconn(x,nlcon1,nlcon2));

if(y_total_max-b<y0_total_max)
    lb = y0_total_max;
else
    lb = y_total_max;
end
[real_pos_M2,real_max_M2] = fmincon(@(x)-fun3(x(1:2),T_norm_func,x0_total_max,y0_total_max), .....
    [x_total_max;y_total_max], [],[],[],[],.....
    [x_total_max-b;lb],.....
    [x_total_max+b;y_total_max+b], @(x)nlconn(x,nlcon1,nlcon2));

% [real_pos_M2,real_max_M2] = fmincon(@(x)-fun4(x(1:2),T2_norm_func,x(3),x(4)), .....
%     [x_total_max;y_total_max;x0_total_max;y0_total_max], [],[],[],[],.....
%     [x_total_max-b;y_total_max-b;x0_total_max-b2;y0_total_max-b3],.....
%     [x_total_max+b;y_total_max+b;x0_total_max+b2;y0_total_max+b3], @(x)nlconn(x,nlcon1,nlcon2));

% [real_pos_M2,real_max_M2] = fmincon(@(x)-fun4(x(1:2),T2_norm_func,x0_total_max,y0_total_max), .....
%     [x_total_max;y_total_max], [],[],[],[],.....
%     [x_total_max-b;y_total_max-b],.....
%     [x_total_max+b;y_total_max+b], @(x)nlconn(x,nlcon1,nlcon2));

r0 = (d2+1)^(1/(2*beta));
theta_0 = pi-asin(((-(3.7/2-1.8/2)-y0_data)/b/r0)^(1/beta));
theta_0 = pi;
[r, theta] = meshgrid([(d1+1)^(1/(2*beta)):0.1:(d2+1)^(1/(2*beta))],[pi/2:0.01:theta_0]);
px = r.*sign(cos(theta)).*abs(cos(theta)).^(1/beta)*a+x0_total_max;
py = r.*sign(sin(theta)).*abs(sin(theta)).^(1/beta)*b+y0_total_max;
sz = size(px);
max = 0;
max2 = 0;
x_max= 0;
y_max= 0;
x2_max= 0;
y2_max= 0;
for i = 1:sz(1)
    for j = 1:sz(2)

        jac(i,j)= T_norm_func(px(i,j),x0_total_max, py(i,j), y0_total_max)*2.5+T2_norm_func(px(i,j),x0_total_max, py(i,j), y0_total_max)*0.1;
        %jac(i,j)= Ad_func(px(i,j),x0_total_max, py(i,j), y0_total_max);
        
    end
end

x02 = x0_total_max;
y02 = y0_total_max;
fh12 = @(x,y)(((x-x02)/a)^2)^beta+(((y-y02)/b)^2)^beta-1-d1;
fh22 = @(x,y)(((x-x02)/a)^2)^beta+(((y-y02)/b)^2)^beta-1-d2;

hold on;
ezplot(fh12,[-40,-20],[-5,5]);
ezplot(fh22,[-40,-20],[-5,5]);
mesh(px,py,jac);
plot(x_total_max,y_total_max,"r*");
plot(real_pos_M2(1),real_pos_M2(2),"b*");


% hold on;
% ezplot(fh1,[-30,-10],[-5,5]);
% ezplot(fh2,[-30,-10],[-5,5]);
% mesh(px,py,jac);
% mesh(px,py,jac2);
% ezplot(fh1,[-30,-10],[-5,5]);
% ezplot(fh2,[-30,-10],[-5,5]);
% mesh(px,py,jac);
% plot([x_max-b:0.01:x_max+b],[x_max-b:0.01:x_max+b]./[x_max-b:0.01:x_max+b]*(y_max-b));
% plot([x_max-b:0.01:x_max+b],[x_max-b:0.01:x_max+b]./[x_max-b:0.01:x_max+b]*(y_max+b));
% plot([y_max-b:0.01:y_max+b]./[y_max-b:0.01:y_max+b]*(x_max+b),[y_max-b:0.01:y_max+b]);
% plot([y_max-b:0.01:y_max+b]./[y_max-b:0.01:y_max+b]*(x_max-b),[y_max-b:0.01:y_max+b]);
% 
% plot(real_pos(1),real_pos(2),"r*");
% plot(x_max,y_max,"b*");
% nlconn([x_max;y_max],nlcon1,nlcon2)
% nlconn(real_pos,nlcon1,nlcon2)

function [c,ceq] = nlconn(x,nlcon1, nlcon2)
    c=[nlcon1(x);nlcon2(x)];
    ceq=0;
end


function c = fun2(x,M_func, x_data, y_data)
    M_max = M_func(x(1),x_data,x(2),y_data);
    [~,c_max,~] = svd(M_max);
    c = c_max(1,1);
end

function c = fun3(x,M_func, x_data, y_data)
    c = abs(M_func(x(1),x_data,x(2),y_data));
end

function c = fun4(x,M_func, x_data, y_data)
    T = M_func(x(1),x_data,x(2),y_data);
    c = sqrt(sum(T.^2));
end

% for temp_d = d1:0.1:4
%     [r, theta] = meshgrid([sqrt(d1):0.1:sqrt(temp_d)],[0:0.01:2*pi]);
%     px = r.*cos(theta)*a+x0;
%     py = r.*cos(theta)*b+y0;
%     sz = size(px);
%     max = 0;
%     x_max= 0;
%     y_max= 0;
%     for i = sz(1)
%         for j = 1:sz(2)
%             M_max = M_func(temp_d, px(i,j), py(i,j));
%             [~,c_max,~] = svd(M_max);
%             c_max = c_max(1,1);
%             if(c_max>max)
%                 max = c_max;
%                 x_max= px(i,j);
%                 y_max= py(i,j);
%             end
%         end
%     end
%     curve_log = [curve_log;max];
% end

