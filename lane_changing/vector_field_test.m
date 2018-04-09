targets=[-10,0,0,0.01;
    10,0,pi/12,0.01]';
[x,y] = meshgrid(-15:0.1:15,-7:0.1:7);
u=x-x;
v=y-y;
sz = size(x);
env.car_w = 2.4/2;
env.min_sep = 0.5;        
for i = 1:sz(1)
    for j = 1:sz(2)
        [vec,~] = merge_vector_field(targets,[x(i,j);y(i,j);0;0.1],env);
        u(i,j) = vec(1);
        v(i,j) = vec(2);
    end
end
hold on
ag = [0:0.1:2*pi];
plot(2.4/2*cos(ag)-10, 2.4/2*sin(ag),'k');
plot(2.4/2*cos(ag)+10, 2.4/2*sin(ag),'k');
quiver(x,y,u,v)
axis equal
