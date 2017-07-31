aviobj=avifile('sim.avi');   %定义一个视频文件用来存动画
aviobj.quality=10;
aviobj.Fps=20;
start_time = 0;
end_time = 4;

hFig = figure;
set(hFig, 'Position', [200 50 800 800]);

t_log = [SIMSTEP:SIMSTEP:SIMLENGTH];

for t = start_time/SIMSTEP+1 : 50 : end_time/SIMSTEP
    
   
%for j=1:SIMLENGTH/TIMESTEP/plot_num-1:SIMLENGTH/TIMESTEP
    %figure;
    axis square;
    %set(gca,'FontSize',20) 
    draw_single_road;
    
    axis([-25, 25, -25, 25]);
    hold on;
    
    draw_rect(d_f_log(t), 0, 2.5, 1, 0, 'b');
    draw_elipse(d_f_log(t), 0, 2.5+d_m, 1, 'r');

    draw_rect(d_e_log(t), 0, 2.5, 1, 0, 'g');
    draw_elipse(d_e_log(t), 0, 2.5+d_m, 1, 'r');
    
    drawnow;
    axis([-25, 25, -25, 25]);
    hold off;
    
    %text=sprintf('t=%0.3fsec',j*TIMESTEP);
    %title(text,'FontSize',20)
    %xlabel('x(m)','FontSize',20) % x-axis label
    %ylabel('y(m)','FontSize',20) % y-axis label
    frame = getframe(gca);
    im=frame2im(frame);
    aviobj=addframe(aviobj,im);
    fprintf('time: %f, in %f\n', (t-1)*SIMSTEP, start_time-end_time);
    %pause(0.1);
    %m(j) = getframe(gca);
end

aviobj=close(aviobj);
%draw_final;