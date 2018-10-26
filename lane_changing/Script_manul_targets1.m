function env = Script_manul_targets1(env, i)

    for k = 1 : env.targets_num
        if(k == 1)
            env.targets(k).u(2) = 0.1;
            
            d = sqrt(sum((env.targets(1).q(1:2)-env.targets(2).q(1:2)).^2));
            if(d < 12)
                env.targets(1).u = [0;1*(d-15)];
            end
            
        else
            env.targets(k).u = env.targets(k).u;
        end
        
        if(k == 7)
            if(i>1000)
                env.targets(k).u(2) = 0;
            end
            if(i>1500)
                env.targets(k).u(2) = -0.1;
            end
        end
    end
end

