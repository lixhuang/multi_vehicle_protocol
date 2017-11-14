T=85;
T_step=0.01;

q=[-3;1;0;0.1];
q_log=[];
t_log=[];
V=0;
V_log=[];

for t=0:T_step:T
    t_log=[t_log;t];
    q_log=[q_log,q];
    V_log=[V_log;V];
    
    r=sqrt(q(1)^2+q(2)^2);
    phi=atan2(q(2),q(1));
    u=1/(r+1)^2*cos(q(3)-phi)*q(4)-4*(q(4)-r/(r+1));
    w=q(4)/r*sin(q(3)-phi)-0.5*wrapToPi(q(3)-phi-pi);
    q_d=[q(4)*cos(q(3));
        q(4)*sin(q(3));
        w;
        u];
        %-q(3);
        %-r*cos(q(3)-phi)-q(4)*(q(1)^2+q(2)^2)];
        %-(q(1)*cos(q(4))+q(2)*cos(q(4)))-(q(1)^2+q(2)^2)/q(4)];
    %u=-r*cos(q(3)-phi)-q(4)*(q(1)^2+q(2)^2)
    %V=q(1)^2+q(2)^2+q(3)^2+q(4)^2;
    %V=q(4)*(q(1)*cos(q(4))+q(2)*cos(q(4))+u);
    q=q+q_d*T_step;
end

q_log=q_log';
plot(q_log(:,1),q_log(:,2));
%figure;
%plot(t_log,V_log);