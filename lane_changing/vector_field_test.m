targets=[-12,0,pi/6,0.01;
    12,0,pi/6,0.01]';
[x,y] = meshgrid(-25:0.5:25,-20:0.5:20);
u=x-x;
v=y-y;
sz = size(x);
for i = 1:sz(1)
    for j = 1:sz(2)
        [vec,~] = merge_vector_field(targets,[x(i,j);y(i,j);0;0.1]);
        u(i,j) = vec(1);
        v(i,j) = vec(2);
    end
end

quiver(x,y,u,v)
