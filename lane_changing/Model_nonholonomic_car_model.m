function q_derivative = Model_nonholonomic_car_model(state,u,param)

% q = [x; y; theta; v];
% u = [beta; u]
    q_derivative = .....
        [state(4)*cos(state(3));
        state(4)*sin(state(3));
        tan(u(1))/param.l*state(4);
        u(2)];
end

