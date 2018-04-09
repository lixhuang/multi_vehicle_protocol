function env = Ctrl_merge_vector_controller1(q, sframe, env, simple_flag)
    if(nargin==3)
        simple_flag = 0;
    end
    %% TODO: change circular to ellipse and add environment set up for collision model
    env.car_w = 2.4/2;
    env.min_sep = 0.5;
    env.blend_width = 5.7-0.5-2.4;
    %% extract environment
    d_min = 2*env.car_w + env.min_sep;

    %% calculate qd and most risky target
    risk_targ_id = 1;
    risk_targ = sframe.targets(1).q;
    risk_d = sum((q(1:2)-risk_targ(1:2)).^2);
    
    %% transfer space with qd
    qd = (sframe.targets(1).q+sframe.targets(2).q)/2;
    
    xbar_d = q(4)*cos(q(3)) - qd(4)*cos(qd(3));
    ybar_d = q(4)*sin(q(3)) - qd(4)*sin(qd(3));
    
    xbar = q(1)-qd(1);
    ybar = q(2)-qd(2);
    vbar = sqrt(xbar_d^2+ybar_d^2);
    thetabar = atan2(ybar_d, xbar_d) - qd(3);
    
    targetsbar = sframe.targets;
    for i = 1:length(sframe.targets)
        targ = sframe.targets(i);
        targxbar_d = targ.q(4)*cos(targ.q(3)) - qd(4)*cos(qd(3));
        targybar_d = targ.q(4)*sin(targ.q(3)) - qd(4)*sin(qd(3));
        targetsbar(i).q(1) = targ.q(1)-qd(1);
        targetsbar(i).q(2) = targ.q(2)-qd(2);
        targetsbar(i).q(4) = sqrt(targxbar_d^2+targybar_d^2);
        targetsbar(i).q(3) = atan2(targybar_d, targxbar_d) - qd(3);
    end
    rqbar = targetsbar(risk_targ_id).q;
    
    r = sqrt(xbar^2+ybar^2);
    if(r < 0.5)
        asdas = 1;
    end
    phi = atan2(ybar,xbar);
    
    %% compute control in relative space
    Ka = -2;
    
    %prepare targets
    targets = [];
    for i = 1:length(sframe.targets)
        targets = [targets, targetsbar(i).q];
    end
    % velocity control
    vbar_d=1/(r+1)^2*cos(thetabar-phi)*vbar-4*(vbar-r/(r+1));
    % angle control
    [vec_ref, theta_ref_d] = merge_vector_field(targets, [xbar; ybar; thetabar; vbar], env);
    theta_ref = atan2(vec_ref(2), vec_ref(1));
    collid_const = 2*(xbar-rqbar(1))*(vbar*cos(thetabar)-rqbar(4)*cos(rqbar(3)))+2*(ybar-rqbar(2))*(vbar*sin(thetabar)-rqbar(4)*sin(rqbar(3)));
    if(collid_const > 0)
        delta = 0;
    else
        delta = 0.5*wrapToPi(thetabar-theta_ref)/(risk_d-d_min)*collid_const;
    end
    thetabar_d=Ka*wrapToPi(thetabar-theta_ref)+theta_ref_d+delta;
    
    r_converg_const = xbar*cos(thetabar)+ybar*sin(thetabar);
    if(vbar > 0.001)
        vbar_d2 = min(-r_converg_const, vbar_d);
        if (vbar_d2 ~= vbar_d)
            asdasd = 1;
        end
        vbar_d = vbar_d2;
    end
    
    %% transfer back control
    a11 = xbar_d*sin(q(3))-ybar_d*cos(q(3));
    a12 = q(4)*sin(q(3))*ybar_d + q(4)*cos(q(3))*xbar_d;
    a21 = xbar_d*cos(q(3))+ybar_d*sin(q(3));
    a22 = q(4)*cos(q(3))*ybar_d - q(4)*sin(q(3))*xbar_d;
    A = [a11, a12; a21, a22];
    uw = A^(-1)*[thetabar_d*(xbar_d^2+ybar_d^2); vbar*vbar_d];
    u = uw(1);
    w = uw(2);
    
    control = [w;u];
    env.u=control;
    if(simple_)
    if(env.i == 1)
        env.thetabar_error_log = zeros([1,length(env.tspan)]);
        env.thetabar_error_log(env.i) = thetabar-theta_ref;
    else
        env.thetabar_error_log(env.i) = thetabar-theta_ref;
    end
    if(env.i == 1)
        env.col_const_log = zeros([1,length(env.tspan)]);
        env.col_const_log(env.i) = delta;
    else
        env.col_const_log(env.i) = delta;
    end
    if(env.i == 1)
        env.derror_log = zeros([1,length(env.tspan)]);
        env.derror_log(env.i) = r;
    else
        env.derror_log(env.i) = r;
    end
end


