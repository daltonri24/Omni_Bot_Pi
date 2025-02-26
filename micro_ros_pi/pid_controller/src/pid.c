#include "pid.h"

struct PIDController {
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
    float prev_error;  // Previous error
    float integral;  // Integral term
};



float PID_step(struct PIDController *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    float proportional = pid->kp * error;

    pid->integral += error;
    float integral = pid->ki * pid->integral;

    float derivative = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;

    float output = proportional + integral + derivative;
    return output;
}