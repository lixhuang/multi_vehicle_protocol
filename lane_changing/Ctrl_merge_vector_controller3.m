function env = Ctrl_merge_vector_controller3(q, sframe, env, simple_flag)
    if(nargin==3)
        simple_flag = 0;
    end
    
    Ka = 7;
    Kv = 4;
    vbar_max = 1.7;
    
    %% overide dm dr, dcwith settings
    
    %% get qd
    qd = env.qd;
    
    
    %% TODO: change circular to ellipse and add environment set up for collision model
    

    
    
    %% transfer space with qd
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
    
    r = sqrt(xbar^2+ybar^2);
    if(r < 0.5)
        bk = 1;
    end
    phi = atan2(ybar,xbar);
    
    %% compute control in relative space
   
    
    %prepare targets
    targets = [];
    for i = 1:length(sframe.targets)
        if(sframe.targets(i).valid)
            targets = [targets, targetsbar(i).q];
        end
    end
    
    % angle control
    [vec_ref, theta_ref_d, sigma, dsigma, reti] = merge_vector_field(targets, [xbar; ybar; thetabar; vbar], env);
    theta_ref = atan2(vec_ref(2), vec_ref(1));
%     collid_const = 2*(xbar-rqbar(1))*(vbar*cos(thetabar)-rqbar(4)*cos(rqbar(3)))+2*(ybar-rqbar(2))*(vbar*sin(thetabar)-rqbar(4)*sin(rqbar(3)));
%     if(collid_const > 0)
%         delta = 0;
%     else
%         delta = 0.5*wrapToPi(thetabar-theta_ref)/(risk_d-d_min)*collid_const;
%     end
    thetabar_d=theta_ref_d - Ka*wrapToPi(thetabar-theta_ref);
    
    % velocity control
    txbar = targetsbar(reti).q(1);
    tybar = targetsbar(reti).q(2);
    tvbar = targetsbar(reti).q(3);
    tthetabar = targetsbar(reti).q(4);
    col_const = cos(tthetabar-atan2(ybar-tybar,xbar-txbar));
    eps_v = 0;
    if(col_const > 0)
        vm = (tvbar+eps_v)*col_const;
    else
        vm = (tvbar-eps_v)*col_const;
    end
    phid_d = ((vbar*sin(thetabar)-tvbar*sin(tthetabar))*(xbar-txbar)-.....
        (vbar*cos(thetabar)-tvbar*cos(tthetabar))*(ybar-tybar))/((ybar-tybar)^2+(xbar-txbar)^2);
    vm_d = tvbar*sin(tthetabar-atan2(ybar-tybar,xbar-txbar))*phid_d;
    eps=0.18;
    vc = vbar_max*(2/(1+exp(-eps*r))-1);
    vc_d = vbar_max*2*eps/(1+exp(-eps*r))^2*exp(-eps*r)*cos(thetabar-phi)*vbar;
    vF = sigma*vc + (1-sigma)*vm;
    vF_d = sigma*vc_d + (1-sigma)*vm_d + dsigma*vc - dsigma*vm;
    vbar_d=vF_d-Kv*(vbar-vF);
    
%     r_converg_const = xbar*cos(thetabar)+ybar*sin(thetabar);
%     if(vbar > 0.001)
%         vbar_d2 = min(-r_converg_const, vbar_d);
%         if (vbar_d2 ~= vbar_d)
%             asdasd = 1;
%         end
%         vbar_d = vbar_d2;
%     end
    
    %% transfer back control
    a11 = xbar_d*sin(q(3))-ybar_d*cos(q(3));
    a12 = q(4)*sin(q(3))*ybar_d + q(4)*cos(q(3))*xbar_d;
    a21 = xbar_d*cos(q(3))+ybar_d*sin(q(3));
    a22 = q(4)*cos(q(3))*ybar_d - q(4)*sin(q(3))*xbar_d;
    A = [a11, a12; a21, a22];
    uw = A^(-1)*[thetabar_d*(xbar_d^2+ybar_d^2); vbar*vbar_d];
    u = uw(1);
    beta = atan(uw(2)*env.model_param.l/q(4));
    
    control = [beta;u];
    env.u=control;
    env.qd = env.qd + env.Ego_dynam(env.qd, [0;0], env.model_param)*env.TIME_STEP;
    if(simple_flag)
        return;
    end
    if(env.i == 1)
        env.thetabar_error_log = zeros([1,length(env.tspan)]);
        env.thetabar_error_log(env.i) = thetabar-theta_ref;
    else
        env.thetabar_error_log(env.i) = thetabar-theta_ref;
    end
    if(env.i == 1)
        env.derror_log = zeros([1,length(env.tspan)]);
        env.derror_log(env.i) = r;
    else
        env.derror_log(env.i) = r;
    end
    if(env.i == 1)
        env.qd_log = zeros([4,length(env.tspan)]);
        env.qd_log(:,env.i) = env.qd;
    else
        env.qd_log(:,env.i) = env.qd;
    end
end



