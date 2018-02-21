% header file for env struct
global TIME_STEP;
global SIM_LENGTH;

global tspan;

global q;
global q_init;
global q_dim;
global q_log;
global u;
global u_log;
global u_dim;
global targets_num;
global targets; %targets is an array of targets object;
% target{q_init, q, q_dim, u}

% u = controller(q, sframe);
global Controller;

% dq = ego_dynam(q,u);
global Ego_dynam;

% dq = target_dynam(q,u);
global Target_dynam;

% sframe = sensing;
global Sensing;

% target_ctrl(i);
global Target_ctrl;








