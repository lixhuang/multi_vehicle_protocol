function env = simple_highway2
    
    %Load_env;
    
    env.TIME_STEP = 0.01;
    env.SIM_LENGTH = 25;
    env.planning_blocking = 50;
    env.tspan = 0;
    env.sens_r = 40;
    
    % for Simple_4states_model
    % x; y; theta; v
    env.q_init = [7;-3.8;0;31.5];
    env.q = env.q_init;
    env.u = [0;0];
    env.q_dim = length(env.q_init);
    env.u_dim = 2;
    
    % car param
    env.model_param.car_w = 2.4/2;
    env.model_param.min_sep = 0.5;
    env.model_param.blend_width = 5-0.5-2.4;
    env.model_param.l = 4.5;
    
    %setup targets
    env.targets_num = 4;
    %right
    env.targets(1).q_init = [-15;-3.7;0;30];
    env.targets(1).q = env.targets(1).q_init;
    env.targets(1).u = [0;0.05];
    env.targets(1).q_dim = length(env.targets(1).q);
    env.targets(1).u_dim = length(env.targets(1).u);
    env.targets(1).valid = 1;
    env.targets(2).q_init = [12;-3.7;0;30];
    env.targets(2).q = env.targets(2).q_init;
    env.targets(2).u = [0;0];
    env.targets(2).q_dim = length(env.targets(2).q);
    env.targets(2).u_dim = length(env.targets(2).u);
    env.targets(2).valid = 1;
    env.targets(3).q_init = [20;0;0;31];
    env.targets(3).q = env.targets(3).q_init;
    env.targets(3).u = [0;0];
    env.targets(3).q_dim = length(env.targets(2).q);
    env.targets(3).u_dim = length(env.targets(2).u);
    env.targets(3).valid = 1;
    env.targets(4).q_init = [-10;0;0;31];
    env.targets(4).q = env.targets(4).q_init;
    env.targets(4).u = [0;-0.2];
    env.targets(4).q_dim = length(env.targets(2).q);
    env.targets(4).u_dim = length(env.targets(2).u);
    env.targets(4).valid = 1;
    
    env.Vr = [1;2];
    env.Vc = [4;3];
    env.Vl = [];

    
    %setup methods
    env.Controller = @Ctrl_simple_highway_controller1; %controller function;
    env.Ego_dynam = @Model_nonholonomic_car_model; %dynamics function;
    env.Target_dynam = @Model_simple_4states_model; %dynamics function;
    env.Sensing = @Sens_deterministic_simple; %sensing function
    env.Target_ctrl = @Script_manul_targets1; %target script function;

end



