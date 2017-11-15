function q_derivative = Model_simple_4states_model(state,u)

% q = [x; y; theta; v];
% u = [w; u]

    q_derivative = .....
        [state(4)*cos(state(3));
        state(4)*sin(state(3));
        u(1);
        u(2)];
end

