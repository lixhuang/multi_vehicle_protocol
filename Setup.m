function ret = Setup(dir, simlate_setup_func)

    addpath(dir);
    addpath(strcat(dir,'/','sim'));
    
    Load_env;
    simlate_setup_func();
    
    tspan = 0:TIME_STEP:SIM_LENGTH;
    
    q_log = zeros([q_dim,length(tspan)]);
    for k = 1:targets_num
        targets(k).q_log = zeros([targets(k).q_dim,length(tspan)]);
    end
    

end

