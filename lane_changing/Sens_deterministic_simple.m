function sframes = Sens_deterministic_simple(env)

    %Load_env;
    
    for k = 1:env.targets_num
        sframes.targets(k).q = env.targets(k).q;
        sframes.targets(k).u = env.targets(k).u;
    end
    sframes.env = [];
end

