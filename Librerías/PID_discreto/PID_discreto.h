#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float Kp, float Ki, float Kd, float dt);
    float compute(float setpoint, float measured);
    void reset();

private:
    float _Kp, _Ki, _Kd, _dt;
    float _previous_error;
    float _integral;
};

#endif
