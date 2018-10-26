function env = Script_manul_targets1(env, i)

    for k = 1 : env.targets_num
        if(k == 1)
            env.targets(k).u(2) = 0.1*cos(i*env.TIME_STEP/pi/2*5);
            
            d = sqrt(sum((env.targets(1).q(1:2)-env.targets(2).q(1:2)).^2));
            if(d < 18)
                env.targets(1).u = [0;3*(d-22)];
            end
            
        else
            env.targets(k).u = env.targets(k).u;
        end
        if(k == 4)
            env.targets(k).u(2) = 0.2*sin(i*env.TIME_STEP/pi/2*2);
            
            d = sqrt(sum((env.targets(3).q(1:2)-env.targets(3).q(1:2)).^2));
            if(d < 18)
                env.targets(1).u = [0;3*(d-22)];
            end
            
        else
            env.targets(k).u = env.targets(k).u;
        end
    end
end

