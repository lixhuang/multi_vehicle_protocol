function y = bump_func(x)
    % y -> 0 when x ->c_min
    % y -> 1 when x ->c_max

    y = 1/(-0.5+0.05)*(x+0.05);
    y(y<0)=0;
    y(y>1)=1;
end

