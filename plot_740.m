figure;
hold on;
yyaxis left;
plot(env.tspan, env.q_log(1,:));
plot(env.tspan, env.qd_log(1,:));
plot(env.tspan, env.targets(1).q_log(1,:));
plot(env.tspan, env.targets(2).q_log(1,:));
ylabel("longitudinal postion on the road (m)")

yyaxis right;
plot(env.tspan, env.q_log(2,:));
ylabel("lateral postion on the road (m)")

xlabel("time (s)")
legend("ego car", "desired postion", "target 1", "target 2");
title("position information")


figure
hold on;
d1 =  sqrt(sum((env.q_log(1:2,:) - env.targets(1).q_log(1:2,:)).^2));
d2 =  sqrt(sum((env.q_log(1:2,:) - env.targets(2).q_log(1:2,:)).^2));
plot(env.tspan, d1);
plot(env.tspan, d2);
plot(env.tspan, 2.9*ones([1,length(env.tspan)]));
ylabel("distance (m)")
xlabel("time (s)")
legend("ego to target 1", "ego to target 2", "min bound")
title("collision information")

figure
hold on;
yyaxis left
plot(env.tspan, env.u_log(1,:));
plot(env.tspan, pi/9*ones([1,length(env.tspan)]));
plot(env.tspan, -pi/9*ones([1,length(env.tspan)]));
ylabel("steering angle \beta (rad)")

yyaxis right
plot(env.tspan, env.u_log(2,:));
plot(env.tspan, 10*ones([1,length(env.tspan)]));
plot(env.tspan, -10*ones([1,length(env.tspan)]));
ylabel("acceleration u (m/s^2)")
xlabel("time (s)")
legend("\beta","ul","lb","u","ul","ub")


