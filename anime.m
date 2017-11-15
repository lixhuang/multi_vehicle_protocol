addpath('drawlib');
figure;
for n = 1:10:length(tspan)
    hold on;
    axis equal;
    draw_st_road;
    for k = 1:targets_num
        draw_car_can(targets(k).q_log(1,n), targets(k).q_log(2,n), targets(k).q_log(3,n));
    end
    draw_car_red(q_log(1,n), q_log(2,n), q_log(3,n));
    
    hold off;
    pause(0.0001);
    clf
end
