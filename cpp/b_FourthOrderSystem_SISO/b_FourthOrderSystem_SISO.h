// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University

#include "PID.h"

template <class T>
void FourthOrderSystem_SISO(T setpoint, T dt, 
         T &prevX1, T &prevX2, T &prevX3, T &prevX4,
         T &prevE, T &prevP, T &prevI, T &prevD, T &prevPID, 
         T Kp, T Ki, T Kd){

    T currPID=(T)0;
    PIDController(setpoint, dt, prevX4,
         prevE, prevP, prevI, prevD, 
         Kp, Ki, Kd, currPID);
    T X_Dot1 = (T)-1*prevX1 + (T)prevPID;  
    T X_Dot2 = (T)-10*prevX2 + (T)10*prevX1;  
    T X_Dot3 = (T)-100*prevX3 + (T)100*prevX2;  
    T X_Dot4 = (T)-1000*prevX4 + (T)1000*prevX3;  
    prevX1 = prevX1 + X_Dot1*dt;
    prevX2 = prevX2 + X_Dot2*dt;
    prevX3 = prevX3 + X_Dot3*dt;
    prevX4 = prevX4 + X_Dot4*dt;
    prevPID = currPID;

}