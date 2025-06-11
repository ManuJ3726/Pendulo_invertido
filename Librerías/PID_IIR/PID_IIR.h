#ifndef PID_IIR_H
#define PID_IIR_H

class PID_IIR {
public:
    PID_IIR(float Kp, float Ki, float Kd, float dt, float u0 = 0);
    float calcular(float setpoint, float valor_medido);
    void reiniciar();

private:
    float _A0, _A1, _A2;
    float _error[3];
    float _salida;
};

#endif
