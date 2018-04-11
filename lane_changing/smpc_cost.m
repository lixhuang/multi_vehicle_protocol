function J = smpc_cost( qd, env )
    N = env.p_horizon;
    ref_blocking = env.ref_blocking;
    
    J = 0;
    
    for case_it = 1:env.case_num
        for i = 1:N
            if(mod(i-1,ref_blocking)==0)
                env.qd = qd(:,floor((i-1)/ref_blocking)+1);
            end
            
            %% calculate control
            sframe = env.Sensing(env); 
            env = env.Controller(env.q, sframe, env, 1);
            % TO DO: need to get target control from env!
            for k = 1 : env.targets_num
                env.targets(k).u = env.targets(k).u;
            end
            
            %% system record
            env.q_log(:,i) = env.q;
            env.u_log(:,i) = env.u;
            for k = 1:env.targets_num
                env.targets(k).q_log(:,i) = env.targets(k).q;
            end
            %% system envolve
            state1 = (env.case_list(case_it,i)-1)/3;
            state2 = mod(env.case_list(case_it,i)-1,3);
            state = [state1;state2];
            env.qd = env.qd + env.Ego_dynam(env.qd, [0;0])*env.TIME_STEP;
            env.q = env.q + env.Ego_dynam(env.q, env.u)*env.TIME_STEP;
            for k = 1:env.targets_num
                env.targets(k).q = env.targets(k).q + .....
                    env.Target_dynam(env.targets(k).q, [0;get_action(state(k))])*env.TIME_STEP;
            end
        end
        
        J = J + env.case_prob(case_it)*sum(sum(env.u_log.^2));
        
    end


end

