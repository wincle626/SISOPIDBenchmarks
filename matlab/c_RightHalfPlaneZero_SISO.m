function c_RightHalfPlaneZero_SISO

% The Multiple Equal Poles benchmark
% Transfer function: G(s) = (1-0.1s)/(s+1)^3
% Created by Yun Wu 2022.01.29
% Copyright @ Yun Wu
% UDRC project
% Heriot-Watt University

%%
clc;
clear;
close all;

%% System parameter
Time = 10;
dt = 0.01;      

%% Matlab implementation with transform function
s = tf('s');
G = (1-0.1*s)/(s+1)^3; % transfer function
[C_opt,info] = pidtune(G,'PID');
C_opt.Kp = C_opt.Kp;
C_opt.Ki = C_opt.Ki;
C_opt.Kd = C_opt.Kd;
C = pid(C_opt.Kp,C_opt.Ki,C_opt.Kd);
T_pid = feedback(C*G, 1);
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
X1 = zeros(1,n+1);
X2 = zeros(1,n+1);
X3 = zeros(1,n+1);
X4 = zeros(1,n+1);
X5 = zeros(1,n+1);
X6 = zeros(1,n+1);
X7 = zeros(1,n+1);
X8 = zeros(1,n+1);
X1_Dot = zeros(1,n+1);
X2_Dot = zeros(1,n+1);
X3_Dot = zeros(1,n+1);
X4_Dot = zeros(1,n+1);
X5_Dot = zeros(1,n+1);
X6_Dot = zeros(1,n+1);
X7_Dot = zeros(1,n+1);
X8_Dot = zeros(1,n+1);
Y1 = zeros(1,n+1);
Y2 = zeros(1,n+1);
Y3 = zeros(1,n+1);
Y = zeros(1,n+1);
X = zeros(1,n+1);
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
    % Plant output (1-0.1s)/(s+1)^3
    % equal to 1/(s+1)^3+(-0.1/(s+1))(1/(s+1))+(-0.1/(s+1))(1/(s+1))(-1/(s+1))
    % sequential
    X1_Dot(i+1) = -X1(i) + U(i);  
    X2_Dot(i+1) = -X2(i) + X1(i); 
    X3_Dot(i+1) = -X3(i) + X2(i);  
    Y1(i+1) = X3_Dot(i+1); 
    X4_Dot(i+1) = -X4(i) - 0.1*U(i);  
    X5_Dot(i+1) = -X5(i) + X4(i); 
    Y2(i+1) = X5_Dot(i+1); 
    X6_Dot(i+1) = -X6(i) - 0.1*U(i);  
    X7_Dot(i+1) = -X7(i) + X6(i); 
    X8_Dot(i+1) = -X8(i) - X7(i);  
    Y3(i+1) = X8_Dot(i+1); 
    Y(i+1) = Y1(i+1) + Y2(i+1) + Y3(i+1);
    % Feedback H(s)=1 
    X1(i+1) = X1(i) + X1_Dot(i+1)*dt; % direct feedback
    X2(i+1) = X2(i) + X2_Dot(i+1)*dt; % direct feedback
    X3(i+1) = X3(i) + X3_Dot(i+1)*dt; % direct feedback
    X4(i+1) = X4(i) + X4_Dot(i+1)*dt; % direct feedback
    X5(i+1) = X5(i) + X5_Dot(i+1)*dt; % direct feedback
    X6(i+1) = X6(i) + X6_Dot(i+1)*dt; % direct feedback
    X(i+1) = X(i) + Y(i+1)*dt;
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