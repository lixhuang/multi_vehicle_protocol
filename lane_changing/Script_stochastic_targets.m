function env = Script_stochastic_targets(env, i)

    %REQUIRE: only three choice for target vehicle: -1, 0, 1
    %EFFECTS: model targets car to be stochastic
    %MODIFIES: env.targets(k).u which is [steering angle; acc]
    markov_freq = 15*(0.1/0.01);
    if(~exist("env.target1_u"))
        env.target1_u = env.targets(1).u;
    end
    if(mod(i, markov_freq)==0)
        
        for k = 1 : env.targets_num
            if(i == 1)
                %set initial value                
                env.targets(k).u = [0;0];
            else
                %probability update
                
                if(i < markov_freq+1)
                    state_last = get_control_class(env.targets(k).u);
                else
                    state_last = get_control_class(env.targets(k).u_log(:,i-1-markov_freq));
                end
                if(i < 2*markov_freq+1)
                    state_last2 = get_control_class(env.targets(k).u);
                else
                    state_last2 = get_control_class(env.targets(k).u_log(:,i-1-2*markov_freq));
                end
                
                val = [-1 0 1];
                P = reshape(env.TM(state_last+1,state_last2+1,:),[1,3]);
                env.targets(k).u = [0;val_select(P,val)];
                if(k==1)
                    env.target1_u = env.targets(k).u;
                end
            end
            
        end
    end

    d = sqrt(sum((env.targets(1).q(1:2)-env.targets(2).q(1:2)).^2));
    if(d < 12)
        env.targets(1).u = [0;5*(d-12)];
    else
        env.targets(1).u = env.target1_u;
    end
end

function sample = val_select(P,X) 
p = cumsum([0; P(1:end-1).'; 1+1e3*eps]); 
[a a] = histc(rand,p); 
sample = X(a);
end