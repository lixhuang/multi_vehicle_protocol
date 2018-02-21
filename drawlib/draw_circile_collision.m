function draw_circile_collision(x,y,theta,v)
    w=2.4;
    ag = [0:0.1:2*pi];
    plot(w/2*cos(ag)+x, w/2*sin(ag)+y,'k');
end

