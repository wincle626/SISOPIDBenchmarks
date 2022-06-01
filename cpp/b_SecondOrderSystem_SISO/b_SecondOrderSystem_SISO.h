// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University

#include "PID.h"

template <class T>
void SecondOrderSystem_SISO(T setpoint, T dt, T &prevX1, T &prevX2,
         T &prevE, T &prevP, T &prevI, T &prevD, T &prevPID, 
         T Kp, T Ki, T Kd){

    T currPID=(T)0;
    PIDController(setpoint, dt, prevX2,
         prevE, prevP, prevI, prevD, 
         Kp, Ki, Kd, currPID);
    T X_Dot1 = (T)-1*prevX1 + prevPID;  
    T X_Dot2 = (T)-10*prevX2 + (T)10*prevX1;  
    prevX1 = prevX1 + X_Dot1*dt;
    prevX2 = prevX2 + X_Dot2*dt;
    prevPID = currPID;

}