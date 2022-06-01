// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University

#include "PID.h"

template <class T>
void RightHalfPlaneZero_SISO(T setpoint, T dt,
         T &prevX1, T &prevX2, T &prevX3, T &prevX4,
         T &prevX5, T &prevX6, T &prevX7, T &prevX8, T &prevX,
         T &prevE, T &prevP, T &prevI, T &prevD, T &prevPID, 
         T Kp, T Ki, T Kd){

    T currPID=(T)0;
    PIDController(setpoint, dt, prevX,
         prevE, prevP, prevI, prevD, 
         Kp, Ki, Kd, currPID);
//     X1_Dot(i+1) = -X1(i) + U(i);  
//     X2_Dot(i+1) = -X2(i) + X1(i); 
//     X3_Dot(i+1) = -X3(i) + X2(i);  
//     Y1(i+1) = X3_Dot(i+1); 
//     X4_Dot(i+1) = -X4(i) - 0.1*U(i);  
//     X5_Dot(i+1) = -X5(i) + X4(i); 
//     Y2(i+1) = X5_Dot(i+1); 
//     X6_Dot(i+1) = -X6(i) - 0.1*U(i);  
//     X7_Dot(i+1) = -X7(i) + X6(i); 
//     X8_Dot(i+1) = -X8(i) - X7(i);  
//     Y3(i+1) = X8_Dot(i+1); 
//     Y(i+1) = Y1(i+1) + Y2(i+1) + Y3(i+1);
//     X1(i+1) = X1(i) + X1_Dot(i+1)*dt; % direct feedback
//     X2(i+1) = X2(i) + X2_Dot(i+1)*dt; % direct feedback
//     X3(i+1) = X3(i) + X3_Dot(i+1)*dt; % direct feedback
//     X4(i+1) = X4(i) + X4_Dot(i+1)*dt; % direct feedback
//     X5(i+1) = X5(i) + X5_Dot(i+1)*dt; % direct feedback
//     X6(i+1) = X6(i) + X6_Dot(i+1)*dt; % direct feedback
//     X7(i+1) = X7(i) + X7_Dot(i+1)*dt; % direct feedback
//     X8(i+1) = X8(i) + X8_Dot(i+1)*dt; % direct feedback
//     X(i+1) = X(i) + Y(i+1)*dt;
    T X_Dot1 = (T)-1*prevX1 + prevPID;  
    T X_Dot2 = (T)-1*prevX2 + prevX1;  
    T X_Dot3 = (T)-1*prevX3 + prevX2;  
    T X_Dot4 = (T)-1*prevX4 - (T)0.1*prevPID;  
    T X_Dot5 = (T)-1*prevX5 + prevX4;  
    T X_Dot6 = (T)-1*prevX6 - (T)0.1*prevPID;  
    T X_Dot7 = (T)-1*prevX7 + prevX6;  
    T X_Dot8 = (T)-1*prevX8 - prevX7;  
    prevX1 = prevX1 + X_Dot1*dt;
    prevX2 = prevX2 + X_Dot2*dt;
    prevX3 = prevX3 + X_Dot3*dt;
    prevX4 = prevX4 + X_Dot4*dt; 
    prevX5 = prevX5 + X_Dot5*dt;
    prevX6 = prevX6 + X_Dot6*dt;
    prevX7 = prevX7 + X_Dot7*dt;
    prevX8 = prevX8 + X_Dot8*dt;
    prevX = prevX + (X_Dot3 + X_Dot5 + X_Dot8)*dt;
    prevPID = currPID;

}