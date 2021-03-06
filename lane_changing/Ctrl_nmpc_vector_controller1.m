function env = Ctrl_smpc_vector_controller1(q, sframe, env)
     
    if(mod(env.i-1,env.planning_blocking)==0)
        env.i
        b_target = env.targets(1);
        b_target.valid = 0;
        J_vec = Inf*ones([env.targets_num+1,1]);
        qd_vec = zeros([env.q_dim+2*15,env.targets_num+1]);
        for it = 1 : env.targets_num+1
            %% prepare environment
            virtual_env.q = q;
            virtual_env.q_dim = env.q_dim;
            virtual_env.u_dim = env.u_dim;
            virtual_env.targets_num = sframe.targets_num;
            virtual_env.targets = sframe.targets;
            virtual_env.mg_id = it;
            
%             virtual_env.mg_targets_num = 2;
%             if(it > env.targets_num)
%                 virtual_env.mg_targets = [b_target;env.targets(1)];
%                 virtual_env.mg_targets(2).valid = 0;
%             else
%                 virtual_env.mg_targets = [b_target;env.targets(it)];
%                 virtual_env.mg_targets(2).valid = 1;
%                 b_target = virtual_env.mg_stargets(2);
%             end
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
            virtual_env.u1_max = pi/9;
            virtual_env.u1_min = -pi/9;
            virtual_env.u2_max = 10;
            virtual_env.u2_min = -10;
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
            virtual_env.case_num = 1;
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

%             virtual_env.case_num = 1;
%             virtual_env.case_prob = [1];
%             virtual_env.case_list = 5*ones([1,virtual_env.p_horizon]);
            if(it == 1)
                qd = sframe.targets(1).q-[10;0;0;0];
            elseif(it > env.targets_num)
                qd = sframe.targets(2).q+[10;0;0;0];
            elseif(virtual_env.targets(1).valid && virtual_env.targets(2).valid)
                qd = 0.5*sframe.targets(1).q+0.5*sframe.targets(2).q;
            end
            
%             if(env.i == 1)
%                 
%             else
%                 %qd = env.qd;
%                 qd2 = env.last_qd_vec(:,it);
%                 [c,ceq] =  smpc_constraints(qd2, virtual_env);
%                 if(~sum(c>0))
%                     qd = qd2;
%                 end
%             end

            %% optimize
            % determine feasibility at first hand
            if(~determine_feasibility(virtual_env))
                J_vec(it) = Inf;               
                continue;
            end
            
            % solve
%             [qd_opt,~,exitflag,~] = fmincon(@(x)smpc_cost(x,virtual_env), qd, [],[],[],[],[],[],.....
%                 @(x)smpc_constraints(x,virtual_env));
            q = [qd;zeros([15*2,1])];
            [qd_opt,~,exitflag,~] = fmincon(@(x)simple_nmpc_cost(x,virtual_env), q, [],[],[],[],[],[],.....
                @(x)simple_nmpc_constraints(x,virtual_env));
            if(exitflag == -2)
                asdasdsd = 1;
            end
            %qd_opt(:,1)
            J_vec(it) = simple_nmpc_cost(qd_opt(:,1),virtual_env);
            qd_vec(:,it) = qd_opt(:,1);
            %env.qd = qd_opt(:,1);
        end
        comp_vec = [J_vec,qd_vec'];
        comp_vec = sortrows(comp_vec);
        env.qd = comp_vec(1,2:5)';
        env.u = comp_vec(1,6:7)';
        env.last_qd_vec = qd_vec;
        if(comp_vec(1,1) == Inf)
            env.qd = [env.q(1);0;0;30];
        end
    end
    
    
    %% calculate control
    %env = Ctrl_merge_vector_controller1(env.q, sframe, env);
    env.qd = env.qd + env.Ego_dynam(env.qd, [0;0], env.model_param)*env.TIME_STEP;
    
%     for it = 1 : env.targets_num+1
%         env.last_qd_vec(:,it) = env.last_qd_vec(:,it) + env.Ego_dynam(env.last_qd_vec(:,it), [0;0], env.model_param)*env.TIME_STEP;
%     end
end

