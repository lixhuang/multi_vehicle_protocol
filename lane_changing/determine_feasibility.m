function res = determine_feasibility(env)

    temp_targs = env.targets;
    d2 = 0;
    res = 1;
    
    for case_it = 1:env.case_num       
        env.targets = temp_targs;
        for i = 1:env.p_horizon            
            %% caluclate constraint on current qd
            if(env.targets(1).valid && env.targets(2).valid)
                v1 = env.targets(2).q(1:2) - env.targets(1).q(1:2);
                d2 = env.d_sep*2+1-sqrt(sum(v1.^2));
            else
                d2 = 0;
            end
            if(d2 > 0)
                res = 0;
                return;
            end
            
            %% system envolve
            state1 = floor((env.case_list(case_it,i)-1)/3);
            state2 = mod(env.case_list(case_it,i)-1,3);
            state = [state1;state2];
            for k = 1:env.targets_num
                if(env.targets(k).valid)
                    env.targets(k).q = env.targets(k).q + .....
                        env.Target_dynam(env.targets(k).q, [0;get_action(state(k))], env.model_param)*env.TIME_STEP;
                end
            end
        end
        
        
        
    end
    
end

