#ifndef PID_BASA_BAJA_H
#define PID_BASA_BAJA_H

class PID_basa_baja {
public:
    PID_basa_baja(float Kp, float Ki, float Kd, float dt, float u0 = 0, float N = 5);
    float calcular(float setpoint, float valor_medido);
    void reiniciar();

private:
    float _A0, _A1;
    float _A0d, _A1d, _A2d;
    float _alpha1, _alpha2;
    float _error[3];
    float _d0, _d1;
    float _fd0, _fd1;
    float _salida;
};

#endif
