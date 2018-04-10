function J = smpc_cost( q_d, env )
    N = env.p_horizon;
    T = env.mpc_Tstep;
    ref_blocking = env.ref_blocking;
    
    J = 0;
    
    for case_it = 1:env.case_num
        for i = 1:N
            env.qd = qd(:,floor((i-1)/ref_blocking)+1);
            
            %% calculate control
            env = env.Controller(env.q, sframe, env);
            % need to get target control from env
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
            env.q + env.Ego_dynam(env.q, env.u)*env.TIME_STEP;
            for k = 1:env.targets_num
                env.targets(k).q = env.targets(k).q + .....
                    env.Target_dynam(env.targets(k).q, env.targets(k).u)*env.TIME_STEP;
            end
        end
        
        J = J + env.case_prob(case_it)*sum(sum(env.u_log.^2));
        
    end


end

