aviobj=VideoWriter('sim.avi');
aviobj.Quality=100;
aviobj.FrameRate=20;
open(aviobj);

hFig = figure;
set(hFig, 'Position', [200 50 800 800]);

for n = 1:10:length(tspan)
    hold on;
    axis equal;
    draw_st_road;
    for k = 1:targets_num
        draw_car_can(targets(k).q_log(1,n), targets(k).q_log(2,n), targets(k).q_log(3,n));
    end
    draw_car_red(q_log(1,n), q_log(2,n), q_log(3,n));
    
    drawnow;
    hold off;
    
    frame = getframe(gca);
    im=frame2im(frame);
    writeVideo(aviobj,im);
    clf
end

close(aviobj);