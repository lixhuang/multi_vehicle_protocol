function env = Script_constant_targets(env, i)

    %Load_env;
    
    for k = 1 : env.targets_num
        env.targets(k).u = env.targets(k).u;
    end
end

