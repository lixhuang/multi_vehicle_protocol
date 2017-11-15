function sframes = Sens_deterministic_simple

    Load_env;
    
    for k = 1:targets_num
        sframes.targets(k).q = targets(k).q;
        sframes.targets(k).u = targets(k).u;
    end
    sframes.env = [];
end

