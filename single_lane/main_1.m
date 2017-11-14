Setup;

d_m=3;
v_d=16.7;
separation_d=10;

%d_f = 12;
%d_e = 0;
x_f = [-15, 16.7, 0];
x_e = [-25, 16.7, 0];
u_f = 0;
u_e = 0;

d_f_log=[];
d_e_log=[];
t_log = [];
t_log = [0:SIMSTEP:SIMLENGTH];
%u_f_profile = 5*sin(1.5*t_log);
u_f_profile = 1.5*(t_log-2).^2-4*1.5;
%u_f_profile = t_log-t_log;

u_f_log = [];
u_e_log = [];

u_e_g_log = [];
delta_log = [];
v_cap_log = [];

u_e_g = 0;
delta = 0;
v_cap = 0;
i=1;
for t = 0:SIMSTEP:SIMLENGTH
    d_f_log = [d_f_log;x_f(1)];
    d_e_log = [d_e_log;x_e(1)];
    u_f_log = [u_f_log;u_f];
    u_e_log = [u_e_log;u_e];
    
    u_e_g_log = [u_e_g_log;u_e_g];
    delta_log = [delta_log;delta];
    v_cap_log = [v_cap_log;v_cap];
    
    u_f = u_f_profile(i);
    if(x_f(2)<=0)
        u_f = 0;
    end
    
    x_f = vehicle_dyn_dintegrat(x_f, u_f, SIMSTEP);
    x_e = vehicle_dyn_dintegrat(x_e, u_e, SIMSTEP);
    x_s = x_f - x_e;
    
    %% assume u_f known
    % nominal PID controller
    % ssperation
    e = separation_d - x_s(1);
    u_e_n = pid_ctrl(e, x_s(2));
    
    % speed
    e = v_d - x_e(2);
    u_e_n = pid_ctrl(e, x_e(3));
    
    
    % guard controller
    if(x_s(1)<d_m)
        fail = 1;
    end
    if(x_e(2)>v_cap)
        fail = 1;
    end
    v_cap = sqrt(x_f(2)^2-2*U_MIN*(x_s(1)-d_m));
    if(x_f(3)<=0)
        u_e_g = (x_f(2)*x_f(3)-U_MIN*x_s(2))/sqrt(x_f(2)^2-2*U_MIN*(x_s(1)-d_m));
    else
        u_e_g = -U_MIN*x_s(2)/sqrt(x_f(2)^2-2*U_MIN*(x_s(1)-d_m));
    end
    c = (x_e(2)-v_cap)/v_cap;
    delta = bump_func(c);
    %delta = 0;
    u_e = delta*u_e_n + (1-delta)*u_e_g;
    
    if(delta == 0)
        alert = 1;
    end
    
    
    i = i+1;
end

save('test_log')

plot(t_log,d_f_log,'b');
hold on;
plot(t_log,d_e_log,'r');
hold off;
disp(fail);

