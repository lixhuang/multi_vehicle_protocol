function env = Script_stochastic_targets(env, i)

    %REQUIRE: only three choice for target vehicle: -1, 0, 1
    %EFFECTS: model targets car to be stochastic
    %MODIFIES: env.targets(k).u which is [steering angle; acc]
  
    for k = 1 : env.targets_num
        if(i == 1)
            %set initial value
            prob =[0.5, 0.25, 0.25;
                  0.25, 0.5, 0.25;
                  0.25, 0.25, 0.5];
            env.targets(k).u = [0;-1];
        else
            %probability update
            prob = prob*prob;
            state_last = env.targets(k).u[2] + 2;
            val = [-1 0 1];
            P = prob(state_last,:);
            env.targets(k).u = [0;val_select(P,val)];              
        end        
    end
end

function sample = val_select(P,X) 
p = cumsum([0; P(1:end-1).'; 1+1e3*eps]); 
[a a] = histc(rand,p); 
sample = X(a);
end