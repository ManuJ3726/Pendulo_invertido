#include "PID_IIR.h"

PID_IIR::PID_IIR(float Kp, float Ki, float Kd, float dt, float u0) {
    _A0 = Kp + Ki * dt + Kd / dt;
    _A1 = -Kp - 2 * Kd / dt;
    _A2 = Kd / dt;
    _error[0] = _error[1] = _error[2] = 0;
    _salida = u0;
}

float PID_IIR::calcular(float setpoint, float valor_medido) {
    _error[2] = _error[1];
    _error[1] = _error[0];
    _error[0] = setpoint - valor_medido;

    _salida += _A0 * _error[0] + _A1 * _error[1] + _A2 * _error[2];
    return _salida;
}

void PID_IIR::reiniciar() {
    _error[0] = _error[1] = _error[2] = 0;
    _salida = 0;
}
