function control = tracing_controller1(q, sframe);
    %designed for Simple_4state_model
    
    qd = sframe.targets(1).q;
    
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
    
    control = [w;u];
end

