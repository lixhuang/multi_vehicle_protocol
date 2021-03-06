clear;
clear global;
%profile on;

%Load_env; % load global variable
env = Setup('lane_changing',@merge2_negative); %load initial settings


for i = 1:length(env.tspan)
    %% generate sensor frame
    env.i = i;
    sframe = env.Sensing(env);
    
    %% calculate control
    env = env.Controller(env.q, sframe, env);
    env = env.Target_ctrl(env,i);
    
    %% system record
    env.q_log(:,i) = env.q;
    env.u_log(:,i) = env.u;
    for k = 1:env.targets_num
        env.targets(k).q_log(:,i) = env.targets(k).q;
        env.targets(k).u_log(:,i) = env.targets(k).u;
    end
    
    %% system envolve
    env.q = env.q + env.Ego_dynam(env.q, env.u, env.model_param)*env.TIME_STEP;
    for k = 1:env.targets_num
        env.targets(k).q = env.targets(k).q + .....
            env.Target_dynam(env.targets(k).q, env.targets(k).u,env.model_param)*env.TIME_STEP;
    end
end

%profile viewer;

%anime;
%gen_video(env,'tracking');

