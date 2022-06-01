
% Created by Yun Wu 2022.01.29
% Copyright @ Yun Wu
% UDRC project
% Heriot-Watt University

function pid = PID_SISO(pid, setpoint, measurement)

    % save state
    pid.setpoint = setpoint;
    pid.measurement = measurement;

    % error signal
    error = setpoint - measurement;
    
    % proportional
    pid.proportional = pid.Kd * error;
    
    % integral
    pid.integrator = pid.integrator ...
                   + 0.5 * pid.Ki * pid.T ...
                   * (error + pid.prevError);
               
    % derivative
    pid.derivative = (2 * pid.Kd * (error - pid.prevError) ...
                   + (2 * pid.tau - pid.T) * pid.derivative) ...
                   / (2 * pid.tau + pid.T);
               
    % pid control signal
    pid.pidoutput = pid.proportional + pid.integrator + pid.derivative;
    
    % save previous error
    pid.prevError = error;
    
end