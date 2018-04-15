function env = Ctrl_smpc_vector_controller1(q, sframe, env)
     
    if(mod(env.i-1,env.planning_blocking)==0)
        env.i
        b_target = env.targets(1);
        b_target.valid = 0;
        for it = 1 : env.targets_num+1
            %% prepare environment
            virtual_env.q = q;
            virtual_env.q_dim = env.q_dim;
            virtual_env.u_dim = env.u_dim;
            %virtual_env.targets_num = sframe.targets_num;
            %virtual_env.targets = sframe.targets;
            virtual_env.targets_num = 2;
            if(it > env.targets_num)
                virtual_env.targets = [b_target;env.targets(1)];
                virtual_env.targets(2).valid = 0;
            else
                virtual_env.targets = [b_target;env.targets(it)];
                virtual_env.targets(2).valid = 1;
                b_target = virtual_env.targets(2);
            end
            virtual_env.model_param = env.model_param;
            
            if(it ~= 2)
                continue;
            end
            
            virtual_env.Controller = @Ctrl_merge_vector_controller1;
            virtual_env.Sensing = @Sens_deterministic_simple;
            virtual_env.Ego_dynam = env.Ego_dynam;
            virtual_env.Target_dynam = env.Target_dynam;

            virtual_env.p_horizon = 45;
            virtual_env.TIME_STEP = 0.1;
            virtual_env.ref_blocking = 45;
            virtual_env.u1_max = pi/6;
            virtual_env.u1_min = -pi/6;
            virtual_env.u2_max = 100;
            virtual_env.u2_min = -100;
            virtual_env.v_max = 1000;
            virtual_env.d_min = 2.9;
            virtual_env.d_sep = 8;

            virtual_env.q_log = zeros([env.q_dim,virtual_env.p_horizon]);
            virtual_env.u_log = zeros([env.u_dim,virtual_env.p_horizon]);
            virtual_env.state_log = zeros([2,virtual_env.p_horizon]);
            for k = 1:env.targets_num
                virtual_env.targets(k).q_log = zeros([env.targets(k).q_dim,virtual_env.p_horizon]);
            end

            %% transition matrix update
            virtual_env.case_num = 5;
            virtual_env.case_blocking = 15;

            state1 = zeros(1,length(sframe.targets));
            state2 = zeros(1,length(sframe.targets));
            for k = 1:length(sframe.targets)
                state1(k) = get_control_class(sframe.targets(k).last_u);
                state2(k) = get_control_class(sframe.targets(k).last2_u);
            end

            

            params.T = zeros(9,9,9);
            for i_1 = 1:3
                for i_2 = 1:3
                    for i_3 = 1:3
                        for j_1 = 1:3
                            for j_2 = 1:3
                                for j_3 = 1:3
                                    params.T((i_1-1)*3 + j_1, (i_2-1)*3 + j_2, (i_3-1)*3 + j_3) = ...
                                        env.TM(i_1,i_2,i_3)*env.TM(j_1,j_2,j_3);
                                end
                            end
                        end
                    end
                end
            end
            params.T_initial_condition = [state2(1)*3+state2(2)+1,  state1(1)*3+state1(2)+1];    % range: [1, size(T,1)]
            params.horizon_total = virtual_env.p_horizon+2*virtual_env.case_blocking;
            params.block_num = virtual_env.case_blocking;
            [T_list, prob_list] = getprob(params);
            weighted_m = [prob_list,T_list(:,2*virtual_env.case_blocking+1:end)];
            weighted_m = sortrows(weighted_m);

            virtual_env.case_prob = weighted_m(end-virtual_env.case_num+1:end,1);
            virtual_env.case_list = weighted_m(end-virtual_env.case_num+1:end,2:end);

    %         virtual_env.case_num = 1;
    %         virtual_env.case_prob = [1];
    %         virtual_env.case_list = 5*ones([1,virtual_env.p_horizon]);

            if(env.i == 1)
                qd = 0.3*sframe.targets(1).q+0.7*sframe.targets(2).q;
            else
                qd = env.qd;
            end

            %% optimize
            qd_opt = fmincon(@(x)smpc_cost(x,virtual_env), qd, [],[],[],[],[],[],.....
                @(x)smpc_constraints(x,virtual_env));
            qd_opt(:,1)
            env.qd = qd_opt(:,1);
        end
        
    end
    
    
    %% calculate control
    
    env = Ctrl_merge_vector_controller1(env.q, sframe, env);
end

