function [c,ceq] = smpc_constraints( qd, env )
    N = env.p_horizon;
    ref_blocking = env.ref_blocking;
    cons_dim = 2*env.u_dim + 1 + env.targets_num;
    cons_dim_t = N*cons_dim;
    
    c = zeros(cons_dim*env.case_num,1);
    
    for case_it = 1:env.case_num
        for i = 1:N
            env.qd = qd(:,floor((i-1)/ref_blocking)+1);
            
            %% calculate control
            sframe = env.Sensing(env);
            env = env.Controller(env.q, sframe, env, 1);
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
            
            %% calculate constraits
            %u v d
            start_idx = (case_it-1)*cons_dim_t+(i-1)*cons_dim;
            c(start_idx+1) = env.u(1)-env.u1_max;
            c(start_idx+2) = -env.u(1)+env.u1_min;
            c(start_idx+3) = env.u(2)-env.u2_max;
            c(start_idx+4) = -env.u(2)+env.u2_min;
            
            c(start_idx+5) = env.q(4)-env.v_max;
            
            for k = 1:env.targets_num
                d = sqrt(sum((env.targets(k).q(1:2) - env.q(1:2)).^2));
                c(start_idx+5+k) = -d+env.d_min;
            end 
        end
        
        
        
    end
    ceq = 0;

end

