function a_MultipleEqualPoles_SISO

% The Multiple Equal Poles benchmark
% Transfer function: G(s) = 1/(s+1)
% Created by Yun Wu 2022.01.29
% Copyright @ Yun Wu
% UDRC project
% Heriot-Watt University

%%
clc;
clear;
close all;

%% System parameter
Time = 5;
dt = 0.01;                                         % sampling time   

%% Matlab implementation with transform function
% s = tf('s','ts',dt);
s = tf('s');
G = 1/(s+1); % transfer function
% [num,den]=tfdata(G);%提取近似传函的分子和分母多项式系数向量
% [A,B,C,D]=tf2ss(num{:},den{:});%转换为状态空间模型
[C_opt,info] = pidtune(G,'PID');
C_opt.Kp = C_opt.Kp;
C_opt.Ki = C_opt.Ki;
C_opt.Kd = C_opt.Kd+0.1;
C = pid(C_opt.Kp,C_opt.Ki,C_opt.Kd);
T_pid = feedback(C*G,1);
% C = C_opt.Kp + C_opt.Ki/s + C_opt.Kd*s;
% T_pid = C*G/(1+C*G);
[y_m,t,~] = step(T_pid, 0:dt:Time);
figure(1);
Reference = info.Stable*ones(1,length(y_m));
plot(t,Reference,'k--');
hold on;
plot(t,y_m,'r--');
axis([min(t) max(t) min(y_m) 1.2*max(y_m)]);
% legend('Target','x_1','location','SouthEast');
grid on;

%% Matlab implementation in state space
n = round(Time/dt);                                 % number of samples
P = zeros(1,n+1); 
I = zeros(1,n+1);
D = zeros(1,n+1); 
U = zeros(1,n+1);
X = zeros(1,n+1);
X_Dot = zeros(1,n+1);
E = zeros(1,n+1);
for i = 1:n
    % Update error
    E(i+1) = info.Stable - X(i);                     % error entering the PID controller  controller  
    % Calculate proportional, derivative, and integration terms
    P(i+1) = E(i+1);                                     % error of proportional term
    I(i+1) = E(i+1)*dt + I(i);                        % integration of the error
    D(i+1) = (E(i+1) - E(i))/dt;                    % derivative of the error
    % PID control output C(s)=Kp+Ki/s+Kd*s
    U(i+1)  = C_opt.Kp*P(i+1) + C_opt.Ki*I(i+1)+ C_opt.Kd*D(i+1);         % the three PID terms
    % Plant output G(s)=1/(s+1) 
    X_Dot(i+1) = -X(i) + U(i);  
    % Feedback H(s)=1 
    X(i+1) = X(i) + X_Dot(i+1)*dt; % direct feedback
end
% figure(2);
T = 0:dt:Time;
% Reference = info.Stable*ones(1,i+1);
% plot(T,Reference,'k--');
% hold on;
plot(T,X,'b--');
xlabel('Time (second)');
ylabel('Amplitude');
% legend('Target','x_1','location','SouthEast');
axis([min(t) max(t) min(X) 1.2*max(X)]);
set(gcf,'color','w');
% grid on;
legend('Target','x_1 TF','x_1 SS','location','SouthEast');

xlm = [4.5, 5];
ylm = [0.99,1.01];
pos = [.4 .3 .2 .3];
magnifyPlot(xlm,ylm,pos,2,false)

set(gcf,'position',[500,500,400,200]);

end