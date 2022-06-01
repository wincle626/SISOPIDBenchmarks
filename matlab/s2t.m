
% Created by Yun Wu 2022.01.29
% Copyright @ Yun Wu
% UDRC project
% Heriot-Watt University

function time_sym = s2t(tf_func)
    % convert the tf object to sym object
    [num,den] = tfdata(tf_func);
    syms s
    t_sym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
    % multiply the input in laplace domain to the transfer function 
    % to get the system response to a specific input in time domain
    time_sym = ilaplace(t_sym/s); % U(s) = 1/s for the unit step
%     G = tf([1], [1 1]);
%     G
%     [num,den] = tfdata(G);
%     syms s
%     G_sym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
%     g_time_sym = ilaplace(G_sym);
%     disp(g_time_sym);
%     Y_lap_sym = G_sym/s; % U(s) = 1/s for the unit step
%     y_time_sym = ilaplace(Y_lap_sym);
%     disp(y_time_sym)
end