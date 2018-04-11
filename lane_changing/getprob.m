% clear, clc;
% 
% %% parameters to be given
% params.T = [0.4 0.6;
%             0.2 0.8];
% 
% params.T_initial_condition = 2;    % range: [1, size(T,1)]
% params.horizon_total = 11;
% params.block_num = 3;
% 
% %% calculate  T_list, prob_list
% 
% [T_list, prob_list] = getProb(params);
% 
% T_list
% prob_list


%% helper function

function [T_list, prob_list] = getProb(params)

T = params.T;
T_initial_condition = params.T_initial_condition;
horizon_total = params.horizon_total;
block_num = params.block_num;

T_size = size(T,1);
horizon = ceil(horizon_total/block_num);
horizon_last = horizon_total - (horizon - 1)*block_num;

% get unrepeated T_list_uni
[T_list_uni, T_list_len] = getlist(T_initial_condition, T_size, horizon);

% get probability for T_list_uni
prob_list = ones(T_list_len,1);
for i = 1:T_list_len
    for j = 2:horizon
        prob_list(i) = prob_list(i) * T(T_list_uni(i,j - 1), T_list_uni(i,j));
    end
end

% Expand T_list_uni to generate T_list
T_list = zeros(T_list_len, horizon_total);
for i = 1:horizon
    if i == horizon
        T_list(:,end - horizon_last + 1:end) = repmat(T_list_uni(:,i),1,horizon_last);
    else
        T_list(:,(i-1)*block_num + (1:block_num)) = repmat(T_list_uni(:,i),1,block_num);
    end
end

end


function [T_list_uni, T_list_len] = getlist(initial, T_size, horizon)

T_list_len = T_size^(horizon - 1);
T_list_uni = zeros(T_list_len, horizon);
T_temp = ones(1, horizon);
T_temp(1) = initial;

% for each possibility
for i = 1 : T_list_len
    % for each digit from backwards
    for j = horizon : -1 : 1
        % è¿›ä½?
        if T_temp(j) > T_size
            T_temp(j) = T_temp(j) - T_size;
            T_temp(j-1) = T_temp(j-1) + 1;
        end
    end
    
    T_list_uni(i,:) = T_temp;
    T_temp(end) = T_temp(end) + 1;
end

end
