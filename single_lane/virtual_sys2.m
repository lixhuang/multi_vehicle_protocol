profile on;
T=15;
T_step=0.01;

% x, y, theta, v
q=[3;1;0;3.1];
qd=[0;0;0;3]

q_log=[];
t_log=[];
qd_log=[];

for t=0:T_step:T
    t_log=[t_log;t];
    q_log=[q_log;q'];
    qd_log=[qd_log;qd'];
    
    xbar_d = q(4)*cos(q(3)) - qd(4)*cos(qd(3));
    ybar_d = q(4)*sin(q(3)) - qd(4)*sin(qd(3));
    
    vbar = sqrt(xbar_d^2+ybar_d^2);
    thetabar = atan2(ybar_d, xbar_d) - qd(3);
    
    r=sqrt((q(1)-qd(1))^2+(q(2)-qd(2))^2);
    phi=atan2(q(2)-qd(2),q(1)-qd(1));
    
    vbar_d=1/(r+1)^2*cos(thetabar-phi)*vbar-4*(vbar-r/(r+1));
    thetabar_d=vbar/r*sin(thetabar-phi)-0.5*wrapToPi(thetabar-phi-pi);
    
    a11 = xbar_d*sin(q(3))-ybar_d*cos(q(3));
    a12 = q(4)*sin(q(3))*ybar_d + q(4)*cos(q(3))*xbar_d;
    a21 = xbar_d*cos(q(3))+ybar_d*sin(q(3));
    a22 = q(4)*cos(q(3))*ybar_d - q(4)*sin(q(3))*xbar_d;
    A = [a11, a12; a21, a22];
    uw = A^(-1)*[thetabar_d*(xbar_d^2+ybar_d^2); vbar*vbar_d];
    u = uw(1);
    w = uw(2);
    
    %if(xbar_d<1e-20 && ybar_d<1e-20)
    %    u = vbar_d;
    %    w = thetabar_d;
    %end
    
    dq=[q(4)*cos(q(3));
        q(4)*sin(q(3));
        w;
        u];
    dqd=[qd(4)*cos(qd(3));
          qd(4)*sin(qd(3));
          -0.0;
          0];
    q=q+dq*T_step;
    qd=qd+dqd*T_step;
end

profile viewer;

hold on;
plot(q_log(:,1),q_log(:,2),'r');
plot(q_log(end,1),q_log(end,2),'r*')

plot(qd_log(:,1),qd_log(:,2), 'b--');
plot(qd_log(end,1),qd_log(end,2), 'bo');
figure;
plot(t_log,q_log(:,3));
plot(t_log,qd_log(:,3));



