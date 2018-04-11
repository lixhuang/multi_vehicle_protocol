function res = get_velocity_class(v)
    %% velocity is classified as (-inf,27],(27,31],(31,inf)
    res = 0;
    if(v<27)
        res = 0;
    elseif(v<31)
        res = 1;
    else
        res = 2;
    end
end

