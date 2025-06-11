#include "PID_basa_baja.h"

PID_basa_baja::PID_basa_baja(float Kp, float Ki, float Kd, float dt, float u0, float N) {
    _A0 = Kp + Ki * dt;
    _A1 = -Kp;

    _A0d = Kd / dt;
    _A1d = -2.0 * Kd / dt;
    _A2d = Kd / dt;

    float tau = Kd / (Kp * N);
    float alpha = dt / (2.0 * tau);

    _alpha1 = alpha / (alpha + 1.0);
    _alpha2 = (alpha - 1.0) / (alpha + 1.0);

    _error[0] = _error[1] = _error[2] = 0;
    _d0 = _d1 = 0;
    _fd0 = _fd1 = 0;
    _salida = u0;
}

float PID_basa_baja::calcular(float setpoint, float valor_medido) {
    _error[2] = _error[1];
    _error[1] = _error[0];
    _error[0] = setpoint - valor_medido;

    // Parte PI
    _salida += _A0 * _error[0] + _A1 * _error[1];

    // Derivada filtrada
    _d1 = _d0;
    _d0 = _A0d * _error[0] + _A1d * _error[1] + _A2d * _error[2];

    _fd1 = _fd0;
    _fd0 = _alpha1 * (_d0 + _d1) - _alpha2 * _fd1;

    _salida += _fd0;

    return _salida;
}

void PID_basa_baja::reiniciar() {
    _error[0] = _error[1] = _error[2] = 0;
    _d0 = _d1 = 0;
    _fd0 = _fd1 = 0;
    _salida = 0;
}
