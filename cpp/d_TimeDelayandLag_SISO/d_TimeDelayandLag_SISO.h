// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University

#include "PID.h"

template <class T>
void TimeDelayandLag_SISO(T setpoint, T dt, T &prevY, T &prevX1, T &prevX,
         T &prevE, T &prevP, T &prevI, T &prevD, T &prevPID, 
         T Kp, T Ki, T Kd){

    T currPID=(T)0;
    PIDController(setpoint, dt, prevX,
         prevE, prevP, prevI, prevD, 
         Kp, Ki, Kd, currPID);
    T X_Dot1 = (T)-10*prevX1 + (T)10*prevPID;  
    prevX1 = prevX1 + X_Dot1*dt;
    prevX = prevX + prevY*dt;
    prevY = X_Dot1;
    prevPID = currPID;

}