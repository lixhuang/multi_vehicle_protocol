function [c,ceq] = smpc_constraints( qd, env )
    N = env.p_horizon;
    ref_blocking = env.ref_blocking;
    cons_dim = 2*env.u_dim + 1 + env.targets_num*2;
    cons_dim_t = N*cons_dim;
    
    c = zeros(cons_dim*env.case_num,1);
    temp_q = env.q;
    temp_targs = env.targets;
    
    for case_it = 1:env.case_num
        env.q = temp_q;
        env.targets = temp_targs;
        for i = 1:N
            if(mod(i-1,ref_blocking)==0)
                env.qd = qd(:,floor((i-1)/ref_blocking)+1);
            end
            
            %% caluclate constraint on current qd
            start_idx = (case_it-1)*cons_dim_t+(i-1)*cons_dim;
            if(env.mg_id == 1)
                v1 = env.qd(1:2) - env.targets(1).q(1:2);
                d2 = [cos(env.targets(1).q(3)), sin(env.targets(1).q(3))]*v1;
                c(start_idx+8) = 1000*(-d2+env.d_sep);
                c(start_idx+9) = -10;
            elseif(env.mg_id > env.targets_num)
                c(start_idx+8) = -10;
                v2 = env.qd(1:2) - env.targets(2).q(1:2);
                d3 = [cos(env.targets(2).q(3)), sin(env.targets(2).q(3))]*v2;
                c(start_idx+9) = 1000*(d3+env.d_sep);
            else
                v1 = env.qd(1:2) - env.targets(1).q(1:2);
                d2 = [cos(env.targets(1).q(3)), sin(env.targets(1).q(3))]*v1;
                c(start_idx+8) = 1000*(-d2+env.d_sep);
                v2 = env.qd(1:2) - env.targets(2).q(1:2);
                d3 = [cos(env.targets(2).q(3)), sin(env.targets(2).q(3))]*v2;
                c(start_idx+9) = 1000*(d3+env.d_sep);
            end
            
            %% calculate control (would envolve qd)
            sframe = env.Sensing(env);
            env = env.Controller(env.q, sframe, env, 1);
            
            %% system record
%             env.q_log(:,i) = env.q;
%             env.u_log(:,i) = env.u;
%             for k = 1:env.targets_num
%                 env.targets(k).q_log(:,i) = env.targets(k).q;
%             end
            
            %% calculate constraits
            %u v d
           
            c(start_idx+1) = env.u(1)-env.u1_max;
            c(start_idx+2) = -env.u(1)+env.u1_min;
            c(start_idx+3) = env.u(2)-env.u2_max;
            c(start_idx+4) = -env.u(2)+env.u2_min;
            
            %c(start_idx+5) = env.q(4)-env.v_max;
            c(start_idx+5) = -10;
            for k = 1:env.targets_num
                if(env.targets(k).valid)
                    d = sqrt(sum((env.targets(k).q(1:2) - env.q(1:2)).^2));
                    c(start_idx+5+k) = -d+env.d_min;
                else
                    c(start_idx+5+k) = -10;
                end
                %c(start_idx+5+k) = -10;
            end 
            
            
            %% system envolve
            state1 = floor((env.case_list(case_it,i)-1)/3);
            state2 = mod(env.case_list(case_it,i)-1,3);
            state = [state1;state2];
            %env.qd = env.qd + env.Ego_dynam(env.qd, [0;0], env.model_param)*env.TIME_STEP;
            env.q = env.q + env.Ego_dynam(env.q, env.u, env.model_param)*env.TIME_STEP;
            for k = 1:env.targets_num
                if(env.targets(k).valid)
                    env.targets(k).q = env.targets(k).q + .....
                        env.Target_dynam(env.targets(k).q, [0;get_action(state(k))], env.model_param)*env.TIME_STEP;
                end
            end
        end
        
        
        
    end
    qd_f = qd(:,end);
    v1 = qd_f(1:2)-env.targets(1).q(1:2);
    v2 = env.targets(2).q(1:2)-env.targets(1).q(1:2);
    ceq = [v1(1)*v2(2)-v1(2)*v2(1);qd_f(3)];

end

