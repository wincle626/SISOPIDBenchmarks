
% Created by Yun Wu 2022.01.29
% Copyright @ Yun Wu
% UDRC project
% Heriot-Watt University

function x = solveeq(A,y)
    x = pinv(A)*y;
end