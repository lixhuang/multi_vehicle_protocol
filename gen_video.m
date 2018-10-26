function gen_video(env, mode)

    if(nargin==1)
        mode='moving_fix';
    end
    addpath('drawlib');
    
    pile_size = 40;
    w_l = 20;
    w_r = 30;

    Load_env;
    
    aviobj=VideoWriter('sim.avi');
    aviobj.Quality=100;
    aviobj.FrameRate=30;
    open(aviobj);

    hFig = figure('units','normalized','outerposition',[0 0 1 1]);
    set(hFig, 'Position', [200 50 800 800]);
    %drawnow;
    %xlim([-20,30]);
    %axis equal;
    %clf
    

    for n = 1:5:length(env.tspan)
        hold on;
        axis equal;
        view_center = env.q_log(1:2,n);
        
        if(strcmp(mode,'tracking'))
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
        end
        
        if(strcmp(mode,'moving_fix'))
            pile_num = floor(view_center(1)/pile_size);
            view_range = [pile_num*pile_size-15,(pile_num+1)*pile_size+15];
            xlim(view_range);
            draw_st_road(view_range);
            for k = 1:env.targets_num
                draw_car_can(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
                draw_circile_collision(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
            end
            draw_car_red(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
            draw_circile_collision(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
        end
        
        if(strcmp(mode,'custom'))
            draw_st_road([1,300]);
            for k = 1:env.targets_num
                draw_car_can(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
                draw_circile_collision(env.targets(k).q_log(1,n), env.targets(k).q_log(2,n), env.targets(k).q_log(3,n));
            end
            draw_car_red(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
            draw_circile_collision(env.q_log(1,n), env.q_log(2,n), env.q_log(3,n));
        end
        
        drawnow;
        axis manual;
        hold off;

        frame = getframe(gcf);
        im=frame2im(frame);
        writeVideo(aviobj,im);
        clf
    end

    close(aviobj);
end