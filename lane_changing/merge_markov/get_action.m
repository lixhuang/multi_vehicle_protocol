function res = get_action(class)
    switch class
        case 0
            res = -1;
        case 1
            res = 0;
        case 2
            res = 1;
        otherwise
            res = 0;
    end
end

