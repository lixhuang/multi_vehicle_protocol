function env = Script_replay_targets(env, i)

    %Load_env;
    if(~exist('env.repaly_log'))
        env_log = load("control_log.mat");
        env.replay_log = env_log.env.targets;
    end
    for k = 1 : env.targets_num
        env.targets(k).u = env.replay_log(k).u_log(:,i);
    end
end
