#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float dt) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _dt = dt;
    _previous_error = 0;
    _integral = 0;
}

float PID::compute(float setpoint, float measured) {
    float error = setpoint - measured;
    _integral += error * _dt;
    float derivative = (error - _previous_error) / _dt;
    float output = _Kp * error + _Ki * _integral + _Kd * derivative;
    _previous_error = error;
    return output;
}

void PID::reset() {
    _integral = 0;
    _previous_error = 0;
}
