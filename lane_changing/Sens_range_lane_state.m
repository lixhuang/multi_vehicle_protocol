function sframes = Sens_record_markov_state(env)

    %Load_env;
    
    %% determine current lane 
    sframes.targets_num = env.targets_num;
    
    for k = 1:env.targets_num
        d_error = env.targets(k).q(1:2) - env.q(1:2);
        
        sframes.targets(k).q = env.targets(k).q;
        sframes.targets(k).u = env.targets(k).u;
        if(sqrt(sum(d_error.^2))<env.sens_r)
            sframes.targets(k).valid = env.targets(k).valid;
        else
            sframes.targets(k).valid = 0;
        end
    end
    
    lane_id = env.q_lane_id;
    sframes.q_lane_id = env.q_lane_id;
    %Vc
    sframes.Vc = [];
    for n = 1:length(env.lane_dic{lane_id})
        targ_num = env.lane_dic{lane_id}(n);
        d = env.q(1)-env.targets(targ_num).q(1);
        if(d < 0)
            sframes.Vc = [n-1;n];
            break;
        end
    end
    if(~isempty(sframes.Vc))
        %back
        b_id = env.lane_dic{lane_id}(sframes.Vc(1));
        if(b_id == 0)
            sframes.Vc(1) = 0;
        elseif(sframes.targets(b_id).valid)
            sframes.Vc(1) = b_id;
        else
            sframes.Vc(1) = 0;
        end
        %front
        f_id = env.lane_dic{lane_id}(sframes.Vc(2));
        if(sframes.targets(f_id).valid)
            sframes.Vc(2) = f_id;
        else
            sframes.Vc(2) = 0;
        end
    elseif(~isempty(env.lane_dic{lane_id}))
        b_id = env.lane_dic{lane_id}(end);
        if(sframes.targets(b_id).valid)
            sframes.Vc = [b_id;0];
        else
            sframes.Vc = [0,0];
        end
    else
        sframes.Vc = [0,0];
    end
    %Vl
    if(lane_id == 1)
        sframes.Vl = [];
    else
        sframes.Vl = env.lane_dic{lane_id-1};
    end
    %Vr
    if(lane_id == env.lane_num)
        sframes.Vr = [];
    else
        sframes.Vr = env.lane_dic{lane_id+1};
    end
    %sframes.env = [];
end