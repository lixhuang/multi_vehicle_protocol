targets=[-7,0,0,0.01;
    10,0,pi/12,0.01]';
[x,y] = meshgrid(-15:0.5:15,-7:0.5:7);
u=x-x;
v=y-y;
sz = size(x);
env.model_param.car_w = 2.4/2;
env.model_param.min_sep = 0.5;
env.model_param.blend_width = 5.7-0.5-2.4;
env.model_param.l = 4.5;

for i = 1:sz(1)
    for j = 1:sz(2)
        [vec,~] = merge_vector_field(targets,[x(i,j);y(i,j);0;0.1],env);
        u(i,j) = vec(1);
        v(i,j) = vec(2);
    end
end
hold on
quiver(x,y,u,v,'g')
ag = [0:0.1:2*pi];
plot(2.4/2*cos(ag)-7, 2.4/2*sin(ag),'k', 'linewidth', 1);
plot(2.4*cos(ag)-7, 2.4*sin(ag),'b--', 'linewidth', 1);
plot(2.9*cos(ag)-7, 2.9*sin(ag),'b--', 'linewidth', 1);
plot(5.7*cos(ag)-7, 5.7*sin(ag),'b--', 'linewidth', 1);
plot(2.4/2*cos(ag)+10, 2.4/2*sin(ag),'k', 'linewidth', 1);

axis equal
