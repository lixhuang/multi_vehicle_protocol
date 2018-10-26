addpath('drawlib');
figure;

subplot(2,2,1);
dr(env,1);
subplot(2,2,2);
dr(env,700);
subplot(2,2,3);
dr(env,1400);
subplot(2,2,4);
dr(env,2500);


function dr(env,n)
    pile_size = 30;
    w_l = 20;
    w_r = 30;

    hold on;
    axis equal;
    
    view_center = env.q_log(1:2,n);
    
    
    
    view_range = [view_center(1)-w_l,view_center(1)+w_r];
    xlim(view_range);
    draw_st_road(view_range);
    for k = 1:env.targets_num
        draw_car_can(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
        draw_circile_collision(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
    end
    draw_car_grey(env.qd_log(1,n), env.qd_log(2,n), env.qd_log(3,n));
    draw_car_red(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
    draw_circile_collision(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
    set(gca,'fontsize',20)
    
%     pile_num = floor(view_center(1)/pile_size);
%     view_range = [pile_num*pile_size-15,(pile_num+1)*pile_size+15];
%     xlim(view_range);
%     draw_st_road(view_range);
%     for k = 1:env.targets_num
%         draw_car_can(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
%         draw_circile_collision(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
%     end
%     if(isfield(env,'qd_log'))
%         draw_car_grey(env.qd_log(1,n), env.qd_log(2,n), env.qd_log(3,n));
%     end
%     draw_car_red(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
%     draw_circile_collision(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
%     %% draw collision model
%     for k = 1:env.targets_num
%         draw_circile_collision(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
%     end
%     draw_circile_collision(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
    
    xlabel("t = "+num2str(env.tspan(n))+"s", 'FontSize', 24);
    drawnow;
    hold off;

end

