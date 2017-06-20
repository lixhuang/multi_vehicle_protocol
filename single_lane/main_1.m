Setup;

k=0.5;
k_d=1;

%d_f = 12;
%d_e = 0;
x_f = [15, 16.7];
x_e = [0, 16.7];
u_f = 0;
u_e = 0;

d_f_log=[];
d_e_log=[];
t_log = [];
t_log = [0:SIMSTEP:SIMLENGTH];
u_f_profile = -200*cos(t_log*5);

u_f_log = [];
u_e_log = [];

i=1;
for t = 0:SIMSTEP:SIMLENGTH
    d = x_f(1) - x_e(1);
    d_f_log = [d_f_log;x_f(1)];
    d_e_log = [d_e_log;x_e(1)];
    u_f_log = [u_f_log;u_f];
    u_e_log = [u_e_log;u_e];
    
    u_f = u_f_profile(i);
    if(x_f(2)<=0)
        u_f = 0;
    end
    
    x_f = vehicle_dyn_dintegrat(x_f, u_f, SIMSTEP);
    x_e = vehicle_dyn_dintegrat(x_e, u_e, SIMSTEP);
    %% assume u_f known
    if(d < 15)
        u_e = -(x_f(2)-x_e(2))^2/2/d+u_f;
    else
        u_e = 0;%x(2)-16.7;
    end
    
    %% assume u_f known spring model
    %u_e = u_f+k*(d-15);
    
    %% assmue u_f known pd model
    u_e = u_f+k*(d-15)+k_d*(x_f(2)-x_e(2));
    
    i = i+1;
end

plot(t_log,d_f_log,'b');
hold on;
plot(t_log,d_e_log,'r');
hold off;
