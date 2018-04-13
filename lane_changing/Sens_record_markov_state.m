function sframes = Sens_record_markov_state(env)

    %Load_env;
    markov_freq = 10*(0.1/0.01); % length of markov decision period
    sframes.targets_num = env.targets_num;
    for k = 1:env.targets_num
        sframes.targets(k).q = env.targets(k).q;
        sframes.targets(k).u = env.targets(k).u;
        if(i < markov_freq+1)
            sframes.targets(k).last_u = env.targets(k).u;
        else
            sframes.targets(k).last_u = env.targets(k).u_log(env.i-markov_freq);
        end
    end
    sframes.env = [];
end