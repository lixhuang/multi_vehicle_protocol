function env = smpc_merge1
    
    %Load_env;
    addpath(strcat('lane_changing/','merge_markov'));
    
    env.TIME_STEP = 0.01;
    env.planning_blocking = 30;
    env.SIM_LENGTH = 15; 
    env.tspan = 0;
    
    % for Simple_4states_model
    % x; y; theta; v
    env.q_init = [-16;0;0;31];
    env.q = env.q_init;
    env.u = [0;0];
    env.q_dim = length(env.q_init);
    env.u_dim = 2;
    
    % car param
    env.model_param.car_w = 2.4/2;
    env.model_param.min_sep = 0.5;
    env.model_param.blend_width = 5.7-0.5-2.4;
    env.model_param.l = 4.5;
    
    %setup targets
    env.targets_num = 2;
    env.targets(1).q_init = [-10;-3.7;0;30];
    env.targets(1).q = env.targets(1).q_init;
    env.targets(1).u = [0;0];
    env.targets(1).q_dim = length(env.targets(1).q);
    env.targets(1).u_dim = length(env.targets(1).u);
    env.targets(1).valid = 1;
    env.targets(2).q_init = [12;-3.7;0;30];
    env.targets(2).q = env.targets(2).q_init;
    env.targets(2).u = [0;0];
    env.targets(2).q_dim = length(env.targets(2).q);
    env.targets(2).u_dim = length(env.targets(2).u);
    env.targets(2).valid = 1;
        
    %Transition model
    env.TM(:,:,1) = [0.1,0.15,0.15;
                0.5,0.1,0.1;
                0.25,0.15,0.2];

    env.TM(:,:,2) = [0.7,0.7,0.6;
                0.4,0.8,0.4;
                0.6,0.7,0.7];

    env.TM(:,:,3) = [0.2,0.15,0.25;
                0.1,0.1,0.5;
                0.15,0.15,0.1];
            
            
    %setup methods
    env.Controller = @Ctrl_smpc_vector_controller1; %controller function;
    env.Ego_dynam = @Model_nonholonomic_car_model; %dynamics function;
    env.Target_dynam = @Model_simple_4states_model; %dynamics function;
    env.Sensing = @Sens_record_markov_state; %sensing function
    env.Target_ctrl = @Script_stochastic_targets; %target script function;

end

