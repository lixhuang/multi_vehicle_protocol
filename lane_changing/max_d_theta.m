function res = max_d_theta(x)
    targets = [-x(4),0,0,0.01]';
    env.car_w = 2.4/2;
    env.min_sep = 0.5;
    [ag,~] = merge_vector_field(targets, [x(1:2);0;1], env);
    ag = atan2(ag(2), ag(1));
    [~,res] = merge_vector_field(targets, [x(1:2);ag;1], env);
    res = -res^2;
end