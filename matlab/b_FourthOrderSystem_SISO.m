function b_FourthOrderSystem_SISO

% The Multiple Equal Poles benchmark
% Transfer function: G(s) = 1/((s+1)(0.1s+1)(0.01s+1)(0.001s+1))
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
dt = 0.01;         

%% Partial fraction decomposition
% from 1/((s+1)(0.1s+1)(0.01s+1)(0.001s+1))
% to a/(s+1)+b/(0.1s+1)+c/(0.01s+1)+d/(0.001s+1)
syms a b c d;
e1 = a + 10*b + 100*c + 1000*d == 0;
e2 = 1.11*a + 11.01*b + 101.1*c + 111*d == 0;
e3 = 0.111*a + 1.011*b + 1.101*c + 1.11*d == 0;
e4 = a + b + c + d == 1;
[a0,b0,c0,d0] = solve(e1,e2,e3,e4,a,b,c,d);
disp([a0,b0,c0,d0])
f1=@(s) 1/((s+1)*(0.1*s+1)*(0.01*s+1)*(0.001*s+1));
f2=@(s) (1000000/890109)/(s+1)+(-10000/8019)/(s+10)+(1000/8019)/(s+100)+(-1000/890109)/(s+1000);
f3=@(s) (1/(s+1))*(10/(s+10))*(100/(s+100))*(1000/(s+1000));
num = rand();
if((f1(num)-f2(num))>0.00001||(f1(num)-f3(num))>0.00001)
    return;
end

%% Matlab implementation with transform function
s = tf('s');
% G = 1/((s+1)*(0.1*s+1)*(0.01*s+1)*(0.001*s+1)); % transfer function
G = (1000000/890109)/(s+1)+(-10000/8019)/(s+10)+...
    (1000/8019)/(s+100)+(-1000/890109)/(s+1000); % transfer function
[C_opt,info] = pidtune(G,'PID');
% [AA,BB,CC,DD] = tf2ss(G.Numerator{:}, G.denominator{:});
% C_opt.Kp = C_opt.Kp*2;
% C_opt.Ki = C_opt.Ki;
% C_opt.Kd = C_opt.Kd+0.1;
C = pid(C_opt.Kp,C_opt.Ki,C_opt.Kd);
T_pid = feedback(C*G, 1);
[y_m,t,~] = step(T_pid,0:dt:Time);
figure(1);
Reference = info.Stable*ones(1,length(y_m));
plot(t,Reference,'k--');
hold on;
plot(t,y_m,'b');
axis([min(t) max(t) min(y_m) 1.2*max(y_m)]);
legend('Target','x_1','location','SouthEast');
grid on;
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
X1_Dot = zeros(1,n+1);
X2_Dot = zeros(1,n+1);
X3_Dot = zeros(1,n+1);
X4_Dot = zeros(1,n+1);
X = zeros(1,n+1);
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
    
    % Plant output G(s)=1/((s+1)(0.1s+1)(0.01s+1)(0.001s+1)) 
    % equal to (1/(s+1))(1/(0.1s+1))(1/(0.01s+1))(1/(0.001s+1))
    % equal to (1/(s+1))(10/(s+10))(100/(s+100))(1000/(s+1000))
    % Parallel
%     X1_Dot(i+1) = -1*X1(i) + (1000000/890109)*U(i); 
%     X2_Dot(i+1) = -10*X2(i) + (-10000/8019)*U(i);  
%     X3_Dot(i+1) = -100*X3(i) + (1000/8019)*U(i); 
%     X4_Dot(i+1) = -1000*X4(i) + (-1000/890109)*U(i);  
%     Y(i+1) = X1_Dot(i+1) + X2_Dot(i+1) + X3_Dot(i+1) + X4_Dot(i+1);  
%     % Sequential
    X1_Dot(i+1) = -X1(i) + U(i);  
    X2_Dot(i+1) = -10*X2(i) + 10*X1(i); 
    X3_Dot(i+1) = -100*X3(i) + 100*X2(i);  
    X4_Dot(i+1) = -1000*X4(i) + 1000*X3(i);    
    Y(i+1) = X4_Dot(i+1);      
    % Feedback H(s)=1
    X1(i+1) = X1(i) + X1_Dot(i+1)*dt; % direct feedback
    X2(i+1) = X2(i) + X2_Dot(i+1)*dt; % direct feedback
    X3(i+1) = X3(i) + X3_Dot(i+1)*dt; % direct feedback
    X4(i+1) = X3(i) + X4_Dot(i+1)*dt; % direct feedback
    X(i+1) = X(i) + Y(i+1)*dt; % direct feedback
end
figure(2);
T = 0:dt:Time;
Reference = info.Stable*ones(1,i+1);
plot(T,Reference,'k--');
hold on;
plot(T,X,'b');
xlabel('Time (second)');
ylabel('Amplitude');
legend('Target','x_1','location','SouthEast');
axis([min(t) max(t) min(X) 1.2*max(X)]);
set(gcf,'color','w');
grid on;

end