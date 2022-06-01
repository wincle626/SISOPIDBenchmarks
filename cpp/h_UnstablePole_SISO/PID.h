// Created by Yun Wu 2022.01.29
// Copyright @ Yun Wu
// UDRC project
// Heriot-Watt University

template <class T>
void PIDController(T setpoint, T dt, T prevX,
         T &prevE, T &prevP, T &prevI, T &prevD, 
         T Kp, T Ki, T Kd, T &PID){

    // Update error
    T currE = setpoint - prevX;
    // Calculate proportional, derivative, and integration terms
    T currP = currE;                    // error of proportional term
    T currI = currE*dt + prevI;         // integration of the error
    T currD = (currE-prevE)/dt;         // derivative of the error
    // PID control output C(s)=Kp+Ki/s+Kd*s
    PID = Kp*currP+Ki*currI+Kd*currD;   // the three PID terms
    // Update intermediate terms
    prevE = currE;
    prevP = currP;
    prevI = currI;
    prevD = currD;

}