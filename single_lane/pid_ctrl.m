function u = Untitled( error, error_d )
% PID controller
    k_p=3;
    k_d=1;
    k_i=0.1;
    k_i_sum_max=20;
    k_i_sum_min=-20;
    persistent k_i_sum;
    if isempty(k_i_sum)
        k_i_sum=0;
    end
    
    k_i_sum = k_i_sum + error;
    if(k_i_sum > k_i_sum_max)
        k_i_sum = k_i_sum_max;
    end
    if(k_i_sum > k_i_sum_min)
        k_i_sum = k_i_sum_min;
    end
    u = k_p*error;% + k_d*error_d + k_i*k_i_sum;
    
    


end

