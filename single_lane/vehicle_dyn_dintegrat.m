function y = vehicle_dyn_dintegrat(x, u, dt)
%   x = [d, v, u]
Setup;

    if(u<U_MIN)
        u=U_MIN;
    end
    if(u>U_MAX)
        u=U_MAX;
    end
    y=[0, 0];
    y(2) = x(2) + dt*u;
    if(y(2)<0)
        y(2)=0;
    end
    y(1) = x(1) + dt*x(2);
    y(3) = u;
end

