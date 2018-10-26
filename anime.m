addpath('drawlib');
figure;
pile_size = 50;
for n = 1:30:length(env.tspan)
    hold on;
    axis equal;
    
    view_center = env.q_log(1:2,n);
    
    pile_num = floor(view_center(1)/pile_size);
    view_range = [pile_num*pile_size-15,(pile_num+1)*pile_size+15];
    xlim(view_range);
    draw_st_road(view_range);
    for k = 1:env.targets_num
        draw_car_can(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
        draw_circile_collision(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
    end
    if(isfield(env,'qd_log'))
        draw_car_grey(env.qd_log(1,n), env.qd_log(2,n), env.qd_log(3,n));
    end
    draw_car_red(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
    draw_circile_collision(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
    %% draw collision model
    for k = 1:env.targets_num
        draw_circile_collision(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
    end
    draw_circile_collision(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
    
    xlabel("t = "+num2str(env.tspan(n))+"s");
    drawnow;
    hold off;
    
    pause(0.0001);
    clf
end
