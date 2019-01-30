function J = smpc_cost( q, env )
    % q:[qd,u]
    qd = q(1:4);
    u_vec = q(5:end);
    N = env.p_horizon;
    ref_blocking = env.ref_blocking;
    
    J = 0;
    J = J + 10*sum((qd(1:2)-env.q(1:2)).^2);
    temp_q = env.q;
    temp_targs = env.targets;
    
    for case_it = 1:env.case_num
        env.q = temp_q;
        env.targets = temp_targs;
        for i = 1:N
            if(mod(i-1,ref_blocking)==0)
                env.qd = qd(:,floor((i-1)/ref_blocking)+1);
            end
            
            %% calculate control
            sframe = env.Sensing(env); 
            %env.u = env.Controller(env.q, sframe, env, 1);
            id = floor((i-1)/3)+1;
            env.u = u_vec(id*2-1:id*2);
            env.qd = env.qd + env.Ego_dynam(env.qd, [0;0], env.model_param)*env.TIME_STEP;
            
            %% system record
%             env.q_log(:,i) = env.q;
             env.u_log(:,i) = env.u;
%             for k = 1:env.targets_num
%                 env.targets(k).q_log(:,i) = env.targets(k).q;
%             end
            
            %% system envolve
            state1 = floor((env.case_list(case_it,i)-1)/3);
            state2 = mod(env.case_list(case_it,i)-1,3);
            state = [state1;state2];
            
            %env.state_log(:,i) = state;
            
            env.q = env.q + env.Ego_dynam(env.q, env.u, env.model_param)*env.TIME_STEP;
            J = J + 10*sum((qd(1:2)-env.q(1:2)).^2);
            for k = 1:env.targets_num
                if(env.targets(k).valid)
                    env.targets(k).q = env.targets(k).q + .....
                        env.Target_dynam(env.targets(k).q, [0;get_action(state(k))], env.model_param)*env.TIME_STEP;
                end
            end
        end
        
        J = J + sum(sum(env.u_log.^2));
        %J = J + 10*(qd(4)-30).^2;
%         hold on
%         plot(env.targets(1).q_log(1,:));
%         plot(env.targets(2).q_log(1,:));
    end


end

