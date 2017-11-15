function sim1
    
    Load_env;
    
    TIME_STEP = 0.01;
    SIM_LENGTH = 11;
    tspan = 0;
    
    % for Simple_4states_model
    % x; y; theta; v
    q_init = [4;0;0;3.1];
    q = q_init;
    q_dim = length(q_init);
    
    %setup targets
    targets_num = 1;
    targets(1).q_init = [-4;-3.7;0;3];
    targets(1).q = targets(1).q_init;
    targets(1).u = [0;0];
    targets(1).q_dim = length(targets(1).q);
    
    %setup methods
    Controller = @Ctrl_tracing_controller1; %controller function;
    Ego_dynam = @Model_simple_4states_model; %dynamics function;
    Target_dynam = @Model_simple_4states_model;; %dynamics function;
    Sensing = @Sens_deterministic_simple; %sensing function
    Target_ctrl = @Script_constant_targets; %target script function;
end

