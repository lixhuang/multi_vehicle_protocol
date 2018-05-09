function env = Ctrl_merge_vector_controller2(q, sframe, env, simple_flag)
    if(nargin==3)
        simple_flag = 0;
    end
    
    env.qd = 0.4*sframe.targets(1).q+0.6*sframe.targets(2).q;
    
    env = Ctrl_merge_vector_controller1(q, sframe, env, simple_flag);
end



