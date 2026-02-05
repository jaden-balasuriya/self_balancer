#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
private:
    float kp, ki, kd;
    float integral;
    float prevError;
    float outputLimit;

public:
    PIDController(float kp, float ki, float kd, float limit = 255.0f)
        : kp(kp), ki(ki), kd(kd),
          integral(0), prevError(0),
          outputLimit(limit) {}

    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        integral += error * dt;
        float derivative = (error - prevError) / dt;
        prevError = error;

        float output =
            kp * error +
            ki * integral +
            kd * derivative;

        if (output > outputLimit) output = outputLimit;
        if (output < -outputLimit) output = -outputLimit;

        return output;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};

#endif
