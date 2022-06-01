function b_ThirdOrderSystem_SISO

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
Time = 6;
dt = 0.01;                                         % sampling time   

%% Matlab implementation with transform function
% s = tf('s','ts',dt);
s = tf('s');
G = 1/((s+1)*(0.1*s+1)*(0.01*s+1)); % transfer function
% G = (1000/891)/(s+1)+(-100/81)/(s+10)+(100/891)/(s+100); % transfer function
f1 = @(v) 1/((v+1)*(0.1*v+1)*(0.01*v+1));
f2 = @(v) (1000/891)/(v+1)+(-100/81)/(v+10)+(100/891)/(v+100);
f3 = @(v) (1/(v+1))*(10/(v+10))*(100/(v+100));
num = randn();
if((f1(num)-f2(num))>0.00001||(f1(num)-f3(num))>0.00001)
    return;
end
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
X1 = zeros(1,n+1);
X2 = zeros(1,n+1);
X3 = zeros(1,n+1);
X_Dot1 = zeros(1,n+1);
X_Dot2 = zeros(1,n+1);
X_Dot3 = zeros(1,n+1);
Y = zeros(1,n+1);
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
    % Plant output G(s)=1/((s+1)(0.1*s+1)*(0.01*s+1))
    % parallel
%     X_Dot1(i+1) = -X1(i) + (1000/891)*U(i);  
%     X_Dot2(i+1) = -10*X2(i) + (-100/81)*U(i); 
%     X_Dot3(i+1) = -100*X3(i) + (100/891)*U(i);  
%     Y(i+1) = X_Dot1(i+1) + X_Dot2(i+1) + X_Dot3(i+1);
    %sequential
    X_Dot1(i+1) = -X1(i) + U(i);  
    X_Dot2(i+1) = -10*X2(i) + 10*X1(i); 
    X_Dot3(i+1) = -100*X3(i) + 100*X2(i);  
    Y(i+1) = X_Dot3(i+1);    
    % Feedback H(s)=1 
    X1(i+1) = X1(i) + X_Dot1(i+1)*dt; % direct feedback
    X2(i+1) = X2(i) + X_Dot2(i+1)*dt; % direct feedback
    X3(i+1) = X3(i) + X_Dot3(i+1)*dt; % direct feedback
    X(i+1) = X(i) + Y(i+1)*dt; % direct feedback
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

end