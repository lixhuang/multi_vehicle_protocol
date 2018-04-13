function res = get_control_class(u)
    %% velocity is classified as (-inf,27],(27,31],(31,inf)
    res = 0;
    if(u(2)<-0.5)
        res = 0;
    elseif(u(2)<0.5)
        res = 1;
    else
        res = 2;
    end
end

