function env = Ctrl_simple_highway_controller1(q, sframe, env)
    
    if(exist("env.qd"))
        qd = env.qd;
    else
        qd = 0;
    end

    u_max = 3;
    dm = 2*ob_sz;
    dr = dm+sep_min;
    dc = dr+env.model_param.blend_width;
    ds = dc+2;

    %% prepare dummy target
    targets = sframe.targets;
    Vr = [env.Vr];
    Vc = env.Vc;
    Vl = [0;env.Vl;0];
    
    virtual_env.q = q;
    virtual_env.q_dim = env.q_dim;
    virtual_env.u_dim = env.u_dim;
    virtual_env.targets_num = sframe.targets_num;
    virtual_env.targets = sframe.targets;

    virtual_env.model_param = env.model_param;
    virtual_env.Controller = @Ctrl_merge_vector_controller3;
    virtual_env.Sensing = @Sens_deterministic_simple;
    virtual_env.Ego_dynam = env.Ego_dynam;
    virtual_env.Target_dynam = env.Target_dynam;

    virtual_env.p_horizon = 40;
    virtual_env.TIME_STEP = 0.05;
    
    t_m = env.planning_blocking*env.TIME_STEP;
    J_min = Inf;
    for k = 1:length(Vr)
        if( k == length(Vr))
            continue;
        end
        vt = targets(Vr(k+1)).q(1:2) - targets(Vr(k+1)).q(1:2);
        vf = targets(Vf).q(1:2) - targets(Vr(k+1)).q(1:2);
        vb = targets(Vb).q(1:2) - targets(Vr(k)).q(1:2);
        
        dist_t = sqrt(sum(vt.^2))-(targets(Vr(k+1)).q(4)-targets(Vr(k)).q(4))*t_m-u_max*t_m^2;
        dist_f = vf'*vt/sqrt(sum(vt.^2));
        dist_b = vb'*vt/sqrt(sum(vt.^2));
        
        if(dist_t > 2*ds && dist_f > 0 && dist_b < 0)
            qd_temp = 0.5*targets(k).q+0.5*targets(k+1).q;
            env.p_horizon = 
            J_temp = simple_cost(qd_temp, virtual_env);
            %J_temp = 0;
            
            %% prepare environment
            
            if(J_temp < J_min)
                qd = qd_temp;
                J_min = J_temp;
            end
        end
    end
    
    env.qd = qd;
    env = Ctrl_merge_vector_controller3(env.q, sframe, env);

    
end

