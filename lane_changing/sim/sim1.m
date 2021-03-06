function env = sim1
    
    %Load_env;
    
    env.TIME_STEP = 0.01;
    env.SIM_LENGTH = 11;
    env.tspan = 0;
    
    % for Simple_4states_model
    % x; y; theta; v
    env.q_init = [-2;0;-atan(0.5);15*sqrt(5)];
    env.q = env.q_init;
    env.u = [0;0];
    env.q_dim = length(env.q_init);
    env.u_dim = 2;
    
    %setup targets
    env.targets_num = 1;
    env.targets(1).q_init = [-4;-3.7;0;15];
    env.targets(1).q = env.targets(1).q_init;
    env.targets(1).u = [0;-0.01];
    env.targets(1).q_dim = length(env.targets(1).q);
    
    %setup methods
    env.Controller = @Ctrl_tracing_controller1; %controller function;
    env.Ego_dynam = @Model_simple_4states_model; %dynamics function;
    env.Target_dynam = @Model_simple_4states_model;; %dynamics function;
    env.Sensing = @Sens_deterministic_simple; %sensing function
    env.Target_ctrl = @Script_constant_targets; %target script function;

end

