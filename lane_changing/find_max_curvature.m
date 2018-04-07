env.car_w = 2.4/2;
env.min_sep = 0.5;

targets=[-10,0,0,0.01;
    10,0,pi/12,0.01]';

t0 = pi/6;
r0=3;
x40 = 8;
x0 = -x40+cos(t0)*r0;
y0 = 0+sin(t0)*r0;
x = fmincon(@max_d_theta, [x0;y0;r0;x40], [], [], [],[],[-inf;-inf;2.4;6],[inf;inf;5.7;20],@nonlc)

function [c,ceq] = nonlc(x)
    ceq = sum((x(1:2)-[-x(4);0]).^2)-x(3)^2;
    c = -1;
end