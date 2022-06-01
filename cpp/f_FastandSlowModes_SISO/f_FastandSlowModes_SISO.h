// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University

#include "PID.h"

template <class T>
void FastandSlowModes_SISO(T setpoint, T dt, T &prevX,
         T &prevX1, T &prevX2, T &prevX3, T &prevX4,
         T &prevE, T &prevP, T &prevI, T &prevD, T &prevPID, 
         T Kp, T Ki, T Kd){

    T currPID=(T)0;
    PIDController(setpoint, dt, prevX,
         prevE, prevP, prevI, prevD, 
         Kp, Ki, Kd, currPID);  
    T X_Dot1 = (T)-10*prevX1 + (T)10*prevPID;  
    T X_Dot2 = (T)-10*prevX2 + (T)10*prevX1;  
    T X_Dot3 = (T)-1*prevX3 + prevX2;  
    T X_Dot4 = (T)-0.05*prevX4 + (T)0.5*prevX2;  
    prevX1 = prevX1 + X_Dot1*dt;
    prevX2 = prevX2 + X_Dot2*dt;
    prevX3 = prevX3 + X_Dot3*dt;
    prevX4 = prevX4 + X_Dot4*dt;
    prevX = prevX + (X_Dot3+X_Dot4)*dt;
    prevPID = currPID;

}