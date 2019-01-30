clear;
clear global;
%profile on;

%Load_env; % load global variable
env = Setup('lane_changing',@close_mpc1); %load initial settings
%env = Setup('lane_changing',@nmpc_merge1); %load initial settings

calt_vec = [];
for i = 1:length(env.tspan)
    %% generate sensor frame
    env.i = i;
    sframe = env.Sensing(env);
    
    %% calculate control
    t1 = tic;
    env = env.Controller(env.q, sframe, env);
    t2 = toc(t1);
    calt_vec = [calt_vec;t2];
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
    if(isfield(env,'q_lane_id'))
        for n = 1:env.lane_num
            if(abs(env.q(2) - env.lane_center(n)) < env.lane_center_eps)
                env.q_lane_id = n;
            end
        end
    end
end

%profile viewer;

%anime;
%gen_video(env,'tracking');

