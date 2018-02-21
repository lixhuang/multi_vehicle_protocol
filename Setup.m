function env = Setup(dir, simulate_setup_func)

    addpath(dir);
    addpath(strcat(dir,'/','sim'));
    
    %Load_env;
    env = simulate_setup_func();
    
    env.tspan = 0:env.TIME_STEP:env.SIM_LENGTH;
    
    env.q_log = zeros([env.q_dim,length(env.tspan)]);
    env.u_log = zeros([env.u_dim,length(env.tspan)]);
    for k = 1:env.targets_num
        env.targets(k).q_log = zeros([env.targets(k).q_dim,length(env.tspan)]);
    end
    

end

