// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University

#include "PID.h"

template <class T>
void MultipleEqualPoles_SISO(T setpoint, T dt, T &prevX,
         T &prevE, T &prevP, T &prevI, T &prevD, T &prevPID, 
         T Kp, T Ki, T Kd){

    T currPID=(T)0;
    PIDController(setpoint, dt, prevX,
         prevE, prevP, prevI, prevD, 
         Kp, Ki, Kd, currPID);
    T X_Dot = (T)-1*prevX + prevPID;  
    prevX = prevX + X_Dot*dt;
    prevPID = currPID;

}