function env = Ctrl_simple_highway_controller1(q, sframe, env)

    if(mod(env.i-1,env.planning_blocking)==0)
        if(isfield(env,"qd"))
            qd = env.qd;
        else
            qd = 0;
        end

        u_max = 3;
        dm = 2*env.model_param.car_w;
        dr = dm+env.model_param.min_sep;
        dc = dr+env.model_param.blend_width;
        ds = dc+1;

        %% prepare dummy target
        targets = sframe.targets;
        dummy = targets(1);
        dummy.q(3) = 0;
        dummy.q(4) = 31;
        
        dummy.q(2) = 0;
        dummy.q(1) = q(1)-env.sens_r;
        targets(sframe.targets_num+1) = dummy;
        dummy.q(2) = 0;
        dummy.q(1) = q(1)+env.sens_r;
        targets(sframe.targets_num+2) = dummy;
        dummy.q(2) = -3.7;
        dummy.q(1) = q(1)-env.sens_r;
        targets(sframe.targets_num+3) = dummy;
        dummy.q(2) = -3.7;
        dummy.q(1) = q(1)+env.sens_r;
        targets(sframe.targets_num+4) = dummy;
        dummy.q(2) = 3.7;
        dummy.q(1) = q(1)-env.sens_r;
        targets(sframe.targets_num+5) = dummy;
        dummy.q(2) = 3.7;
        dummy.q(1) = q(1)+env.sens_r;
        targets(sframe.targets_num+6) = dummy;

        Vr = [sframe.targets_num+3;env.Vr;sframe.targets_num+4];
        Vc = env.Vc;
        if(Vc(1) == 0)
            Vc(1) = sframe.targets_num+1;
        end
        if(Vc(2) == 0)
            Vc(2) = sframe.targets_num+2;
        end
        Vl = [sframe.targets_num+5;env.Vl;sframe.targets_num+6];

        virtual_env.q = q;
        virtual_env.q_dim = env.q_dim;
        virtual_env.u_dim = env.u_dim;
        virtual_env.targets_num = sframe.targets_num;
        virtual_env.targets = sframe.targets;

        virtual_env.model_param = env.model_param;
        virtual_env.Controller = @Ctrl_merge_vector_controller3;
        virtual_env.Sensing = @Sens_deterministic_simple;
        virtual_env.Ego_dynam = env.Ego_dynam;
        virtual_env.Target_dynam = env.Target_dynam;

        virtual_env.p_horizon = 50;
        virtual_env.TIME_STEP = 0.05;
        
        t_m = env.planning_blocking*env.TIME_STEP;
        J_min = Inf;
        for k = 1:length(Vr)
            if( k == length(Vr))
                continue;
            end
            vt = targets(Vr(k+1)).q(1:2) - targets(Vr(k)).q(1:2);
            vf = -(targets(Vc(2)).q(1:2) - targets(Vr(k+1)).q(1:2));
            vb = -(targets(Vc(1)).q(1:2) - targets(Vr(k)).q(1:2));

            dist_t = sqrt(sum(vt.^2))-(targets(Vr(k+1)).q(4)-targets(Vr(k)).q(4))*t_m-u_max*t_m^2;
            dist_f = vf'*vt/sqrt(sum(vt.^2));%-(targets(Vc(2)).q(4)-targets(Vr(k)).q(4))*t_m-u_max*t_m^2;
            dist_b = vb'*vt/sqrt(sum(vt.^2));%-(targets(Vr(k+1)).q(4)-targets(Vc(1)).q(4))*t_m-u_max*t_m^2;

            if(dist_t > 2*ds && dist_f >= 0 && dist_b <= 0)
                qd_temp = 0.5*targets(Vr(k)).q+0.5*targets(Vr(k+1)).q;
                if(k==1 && length(Vr)~=0)
                    qd_temp = targets(Vr(k+1)).q;
                    qd_temp(1) = qd_temp(1)-13;
                end
                if(k==length(Vr)-1 && length(Vr)~=0)
                    qd_temp = targets(Vr(k)).q;
                    qd_temp(1) = qd_temp(1)+13;
                end
                virtual_env.qd = qd_temp;
                J_temp = simple_cost(qd_temp, virtual_env);
                %J_temp = 0;

                %% prepare environment

                if(J_temp < J_min)
                    qd = qd_temp;
                    J_min = J_temp;
                end
            end
        end
        
%         if(env.i>1700)
%             asdas=1;
%         end
        
        for k = 1:length(Vl)
            if( k == length(Vl))
                continue;
            end
            vt = targets(Vl(k+1)).q(1:2) - targets(Vl(k)).q(1:2);
            vf = -(targets(Vc(2)).q(1:2) - targets(Vl(k+1)).q(1:2));
            vb = -(targets(Vc(1)).q(1:2) - targets(Vl(k)).q(1:2));

            dist_t = sqrt(sum(vt.^2))-(targets(Vl(k+1)).q(4)-targets(Vl(k)).q(4))*t_m-u_max*t_m^2;
            dist_f = vf'*vt/sqrt(sum(vt.^2));
            dist_b = vb'*vt/sqrt(sum(vt.^2));

            if(dist_t > 2*ds && dist_f >= 0 && dist_b <= 0)
                qd_temp = 0.5*targets(Vl(k)).q+0.5*targets(Vl(k+1)).q;
                if(k==1 && length(Vl)~=0)
                    qd_temp = targets(Vl(k+1)).q;
                    qd_temp(1) = qd_temp(1)-15;
                end
                if(k==length(Vl)-1 && length(Vl)~=0)
                    qd_temp = targets(Vl(k)).q;
                    qd_temp(1) = qd_temp(1)+15;
                end
                
                virtual_env.qd = qd_temp;
                J_temp = simple_cost(qd_temp, virtual_env);
                %J_temp = 0;

                %% prepare environment

                if(J_temp < J_min)
                    qd = qd_temp;
                    J_min = J_temp;
                end
            end
        end    
        

        vt = targets(Vc(2)).q(1:2) - targets(Vc(1)).q(1:2);

        dist_t = sqrt(sum(vt.^2))-(targets(Vc(2)).q(4)-targets(Vc(1)).q(4))*t_m-u_max*t_m^2;
        dist_t = inf;
        if(dist_t > 2*ds)
            qd_temp = 0.5*targets(Vc(2)).q+0.5*targets(Vc(1)).q;
            virtual_env.qd = qd_temp;
            J_temp = simple_cost(qd_temp, virtual_env);
            %J_temp = 0;

            %% prepare environment

            if(J_temp < J_min)
                qd = qd_temp;
                J_min = J_temp;
            end
        end   
        
        env.qd = qd;
    end
    env = Ctrl_merge_vector_controller3(env.q, sframe, env);

    
end

