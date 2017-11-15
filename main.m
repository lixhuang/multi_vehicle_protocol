clear;
clear global;
profile on;

Load_env; % load global variable
Setup('lane_changing',@sim1); %load initial settings


for i = 1:length(tspan)
    %% system record
    q_log(:,i) = q;
    for k = 1:targets_num
        targets(k).q_log(:,i) = targets(k).q;
    end
    
    %% generate sensor frame
    sframe = Sensing();
    
    %% calculate control
    u = Controller(q, sframe);
    Target_ctrl(i);
    
    %% system envolve
    q = q + Ego_dynam(q, u)*TIME_STEP;
    for k = 1:targets_num
        targets(k).q = targets(k).q + Target_dynam(targets(k).q, targets(k).u)*TIME_STEP;
    end
end

profile viewer;

gen_video;
%hold on;
%plot(q_log(1,:),q_log(2,:),'r');
%plot(q_log(1,end),q_log(2,end),'r*')

%plot(targets(1).q_log(1,:),targets(1).q_log(2,:), 'b--');
%plot(targets(1).q_log(1,end),targets(1).q_log(2,end), 'bo');

