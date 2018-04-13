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
        virtual_env.u2_max = 100;
        virtual_env.u2_min = -100;
        virtual_env.v_max = 1000;
        virtual_env.d_min = 4;
        virtual_env.d_sep = 8;

        virtual_env.q_log = zeros([env.q_dim,virtual_env.p_horizon]);
        virtual_env.u_log = zeros([env.u_dim,virtual_env.p_horizon]);
         virtual_env.state_log = zeros([2,virtual_env.p_horizon]);
        for k = 1:env.targets_num
            virtual_env.targets(k).q_log = zeros([env.targets(k).q_dim,virtual_env.p_horizon]);
        end

        %% transition matrix update
        virtual_env.case_num = 5;
        virtual_env.case_blocking = 10;
        
        state = zeros(1,length(sframe.targets));
        for k = 1:length(sframe.targets)
            state(k) = get_velocity_class(sframe.targets(k).q(4));
        end
        
        TM(:,:,1) = [0.05,0.15,0.8;
                    0.05,0.15,0.8;
                    0.05,0.15,0.8];
                
        TM(:,:,2) = [0.2,0.6,0.2;
                    0.2,0.6,0.2;
                    0.2,0.6,0.2];
                
        TM(:,:,3) = [0.7,0.2,0.1;
                    0.7,0.2,0.1;
                    0.7,0.2,0.1];
                
        params.T = zeros(9,9,9);
        for i_1 = 1:3
            for i_2 = 1:3
                for i_3 = 1:3
                    for j_1 = 1:3
                        for j_2 = 1:3
                            for j_3 = 1:3
                                params.T((i_1-1)*3 + j_1, (i_2-1)*3 + j_2, (i_3-1)*3 + j_3) = ...
                                    TM(i_1,i_2,i_3)*TM(j_1,j_2,j_3);
                            end
                        end
                    end
                end
            end
        end
        params.T_initial_condition = [state(1)*3+state(2)+1,  state(1)*3+state(2)+1];    % range: [1, size(T,1)]
        params.horizon_total = virtual_env.p_horizon+virtual_env.case_blocking;
        params.block_num = virtual_env.case_blocking;
        [T_list, prob_list] = getprob(params);
        weighted_m = [prob_list,T_list(:,2*virtual_env.case_blocking+1:end)];
        weighted_m = sortrows(weighted_m);
        
        virtual_env.case_prob = weighted_m(end-virtual_env.case_num:end,1);
        virtual_env.case_list = weighted_m(end-virtual_env.case_num:end,2:end);
        
        virtual_env.case_num = 1;
        virtual_env.case_prob = [1];
        virtual_env.case_list = 5*ones([1,virtual_env.p_horizon]);
        
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

