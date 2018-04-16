function sframes = Sens_record_markov_state(env)

    %Load_env;
    markov_freq = 15*(0.1/0.01); % length of markov decision period
    sframes.targets_num = env.targets_num;
    for k = 1:env.targets_num
        sframes.targets(k).q = env.targets(k).q;
        sframes.targets(k).u = env.targets(k).u;
        sframes.targets(k).valid = env.targets(k).valid;
        if(env.i < markov_freq+1)
            sframes.targets(k).last_u = env.targets(k).u;
        else
            sframes.targets(k).last_u = env.targets(k).u_log(:,env.i-markov_freq);
        end
        if(env.i < 2*markov_freq+1)
            sframes.targets(k).last2_u = env.targets(k).u;
        else
            sframes.targets(k).last2_u = env.targets(k).u_log(:,env.i-2*markov_freq);
        end
    end
    sframes.env = [];
end