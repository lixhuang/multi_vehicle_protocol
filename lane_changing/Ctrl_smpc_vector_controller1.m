function env = Ctrl_smpc_vector_controller1(q, sframe, env)
     
    if(mod(env.i-1,env.planning_blocking)==0)
        env.i
        %% prepare environment
        virtual_env.q = q;
        virtual_env.q_dim = env.q_dim;
        virtual_env.u_dim = env.u_dim;
        virtual_env.targets_num = sframe.targets_num;
        virtual_env.targets = sframe.targets;

        virtual_env.model_param = env.model_param;

        virtual_env.Controller = @Ctrl_merge_vector_controller1;
        virtual_env.Sensing = env.Sensing;
        virtual_env.Ego_dynam = env.Ego_dynam;
        virtual_env.Target_dynam = env.Target_dynam;

        virtual_env.p_horizon = 30;
        virtual_env.TIME_STEP = 0.1;
        virtual_env.ref_blocking = 30;
        virtual_env.u1_max = pi/6;
        virtual_env.u1_min = -pi/6;
        virtual_env.u2_max = 6;
        virtual_env.u2_min = -6;
        virtual_env.v_max = 1000;
        virtual_env.d_min = 4;
        virtual_env.d_sep = 8;

        virtual_env.q_log = zeros([env.q_dim,virtual_env.p_horizon]);
        virtual_env.u_log = zeros([env.u_dim,virtual_env.p_horizon]);

        %% transition matrix update
        virtual_env.case_num = 1;
        virtual_env.case_blocking = 30;
        virtual_env.case_prob=[1];
        qd = (sframe.targets(1).q+sframe.targets(2).q)/2;

        %% optimize
        qd_opt = fmincon(@(x)smpc_cost(x,virtual_env), qd, [],[],[],[],[],[],.....
            @(x)smpc_constraints(x,virtual_env));
        qd_opt(:,1)
        env.qd = qd_opt(:,1);
    end
    
    %% calculate control
    
    env = Ctrl_merge_vector_controller1(env.q, sframe, env);
end

