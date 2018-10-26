% figure;
% hold on;
% yyaxis left;
% plot(env.tspan, env.q_log(1,:));
% plot(env.tspan, env.qd_log(1,:));
% plot(env.tspan, env.targets(1).q_log(1,:));
% plot(env.tspan, env.targets(2).q_log(1,:));
% ylabel("longitudinal postion on the road (m)")
% 
% yyaxis right;
% plot(env.tspan, env.q_log(2,:));
% ylabel("lateral postion on the road (m)")
% 
% xlabel("time (s)")
% legend("ego car", "desired postion", "target 1", "target 2");
% title("position information")


figure
hold on;
d1 =  sqrt(sum((env.q_log(1:2,:) - env.targets(1).q_log(1:2,:)).^2));
d2 =  sqrt(sum((env.q_log(1:2,:) - env.targets(2).q_log(1:2,:)).^2));
d3 =  sqrt(sum((env.q_log(1:2,:) - env.targets(3).q_log(1:2,:)).^2));
d4 =  sqrt(sum((env.q_log(1:2,:) - env.targets(4).q_log(1:2,:)).^2));
d5 =  sqrt(sum((env.q_log(1:2,:) - env.targets(5).q_log(1:2,:)).^2));
d6 =  sqrt(sum((env.q_log(1:2,:) - env.targets(6).q_log(1:2,:)).^2));
d7 =  sqrt(sum((env.q_log(1:2,:) - env.targets(7).q_log(1:2,:)).^2));
plot(env.tspan, d1, 'LineWidth',1.5);
plot(env.tspan, d2, 'LineWidth',1.5);
plot(env.tspan, d3, 'LineWidth',1.5);
plot(env.tspan, d4, 'LineWidth',1.5);
plot(env.tspan, d5, 'LineWidth',1.5);
plot(env.tspan, d6, 'LineWidth',1.5);
plot(env.tspan, d7, 'LineWidth',1.5);
plot(env.tspan, 2.9*ones([1,length(env.tspan)]), 'LineWidth',1.5);
set(gca,'fontsize',20)
ylabel("distance (m)", 'FontSize', 24)
xlabel("time (s)", 'FontSize', 24)
ylim([0,20]);
lgd = legend("ego to target 1", "ego to target 2", "ego to target 3","ego to target 4",.....
    "ego to target 5", "ego to target 6", "ego to target 7", "min bound")
lgd.FontSize = 16;
title("collision information", 'FontSize', 24)

% figure
% hold on;
% yyaxis left
% plot(env.tspan, env.u_log(1,:));
% %plot(env.tspan, pi/9*ones([1,length(env.tspan)]));
% %plot(env.tspan, -pi/9*ones([1,length(env.tspan)]));
% ylabel("steering angle \beta (rad)")
% 
% yyaxis right
% plot(env.tspan, env.u_log(2,:));
% %plot(env.tspan, 10*ones([1,length(env.tspan)]));
% %plot(env.tspan, -10*ones([1,length(env.tspan)]));
% ylabel("acceleration u (m/s^2)")
% xlabel("time (s)")
% %legend("\beta","ul","lb","u","ul","ub")
% legend("\beta","u")


figure
hold on;
plot(env.tspan, env.u_log(1,:), 'LineWidth',1.5);
%plot(env.tspan, pi/9*ones([1,length(env.tspan)]));
%plot(env.tspan, -pi/9*ones([1,length(env.tspan)]));
set(gca,'fontsize',20)
ylabel("steering angle \beta (rad)", 'FontSize', 24)
xlabel("time (s)", 'FontSize', 24)
lgd = legend("\beta");
lgd.FontSize = 16;

figure;
plot(env.tspan, env.u_log(2,:), 'LineWidth',1.5);
set(gca,'fontsize',16)
ylabel("acceleration u (m/s^2)", 'FontSize', 24)
xlabel("time (s)", 'FontSize', 24)
lgd = legend("u");
lgd.FontSize = 16;

