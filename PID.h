#ifndef _PID_H_
#define _PID_H_

#include "ArduinoShim.h"

typedef struct PID{
    float min_val;
    float max_val;
    float kp;
    float ki;
    float kd;
    double integral;
    double derivative;
    double prev_error;
}PID;

void newPID(PID *pid, float min_val, float max_val, float kp, float ki, float kd)
{
    pid->min_val = min_val;
    pid->max_val = max_val;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0;
    pid->derivative = 0;
    pid->prev_error = 0;
}

double computePID(PID *pid, float setpoint, float measured_value)
{
    double error;
    double pid_out;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    pid->integral += error;
    pid->derivative = error -  pid->prev_error;

    if(setpoint == 0 && error == 0)
    {
        pid->integral = 0;
    }

    pid_out = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * pid->derivative);
    pid->prev_error = error;

    return constrain(pid_out, pid->min_val, pid->max_val);
}

void updatePIDConstants(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

#endif