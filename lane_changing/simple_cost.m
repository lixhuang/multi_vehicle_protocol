function J = simple_cost( qd, env )

    N = env.p_horizon;
    J = 0;
    
    for i = 1:N
        %% calculate control
        sframe = env.Sensing(env); 
        env = env.Controller(env.q, sframe, env, 1);

        %% system record
        env.u_log(:,i) = env.u;

        %% system envolve
        env.q = env.q + env.Ego_dynam(env.q, env.u, env.model_param)*env.TIME_STEP;
        for k = 1:env.targets_num
            if(env.targets(k).valid)
                env.targets(k).q = env.targets(k).q + .....
                    env.Target_dynam(env.targets(k).q, [0;0], env.model_param)*env.TIME_STEP;
            end
        end
        J = J + 1000*(env.q(4)-31)^2;
    end
    
    J = J + sum(sum(env.u_log.^2));


end

