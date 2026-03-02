// pid.h — PID controller for Arduino (dt in seconds) with output limits,
// anti-windup (back-calculation), derivative-on-measurement, and D low-pass.

#ifndef PID_H
#define PID_H

class PID {
public:
  // outMax/outMin should be whatever your motor mapper expects (e.g. -100..100)
  PID(float kp, float ki, float kd,
      float outMax = 1e9f, float outMin = -1e9f,
      float dFilterHz = 25.0f, float awGain = 0.5f)
    : _kp(kp), _ki(ki), _kd(kd),
      _outMax(outMax), _outMin(outMin),
      _dFilterHz(dFilterHz), _awGain(awGain) {
    if (_outMax < _outMin) { float t = _outMax; _outMax = _outMin; _outMin = t; }
  }

  void setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
  }

  // Keep signature as you requested: (outMax, outMin)
  void setOutputLimits(float outMax, float outMin) {
    _outMax = outMax;
    _outMin = outMin;
    if (_outMax < _outMin) { float t = _outMax; _outMax = _outMin; _outMin = t; }
  }

  // Optional: adjust derivative filter (Hz) and anti-windup strength
  void setDerivativeFilterHz(float hz) { _dFilterHz = (hz > 0.0f) ? hz : 0.0f; }
  void setAntiWindupGain(float g)      { _awGain = (g >= 0.0f) ? g : 0.0f; }

  void reset(float currentInput = 0.0f) {
    _integral = 0.0f;
    _dFilt = 0.0f;
    _lastInput = currentInput;
    _hasLast = false;
  }

  // dt in seconds
  float compute(float setpoint, float input, float dt) {
    // dt guards (prevents spikes on first loop or timing hiccups)
    if (dt < 1e-4f) dt = 1e-4f;   // 0.1 ms
    if (dt > 0.05f) dt = 0.05f;   // 50 ms

    const float error = setpoint - input;

    // Derivative on measurement (reduces derivative kick)
    float dMeas = 0.0f;
    if (_hasLast) {
      dMeas = (input - _lastInput) / dt; // measurement derivative
    } else {
      _hasLast = true;
      _lastInput = input;
      dMeas = 0.0f;
    }

    // 1st-order low-pass on derivative (optional but recommended for IMU noise)
    float dTerm = 0.0f;
    if (_dFilterHz > 0.0f) {
      const float rc = 1.0f / (2.0f * 3.1415926f * _dFilterHz);
      const float alpha = dt / (rc + dt);           // 0..1
      _dFilt += alpha * (dMeas - _dFilt);
      dTerm = -_kd * _dFilt;                        // negative because D on measurement
    } else {
      dTerm = -_kd * dMeas;
    }

    // PI + D
    float u = (_kp * error) + (_ki * _integral) + dTerm;

    // Clamp
    float uSat = u;
    if (uSat > _outMax) uSat = _outMax;
    if (uSat < _outMin) uSat = _outMin;

    // Anti-windup: back-calculation (integrator tracks saturation)
    // integral_dot = error + awGain*(uSat - u)
    if (_ki != 0.0f) {
      _integral += (error + _awGain * (uSat - u)) * dt;
    } else {
      _integral = 0.0f;
    }

    _lastInput = input;
    return uSat;
  }

private:
  float _kp, _ki, _kd;

  float _integral = 0.0f;

  float _lastInput = 0.0f;
  bool  _hasLast = false;

  // Derivative filter state/params
  float _dFilt = 0.0f;
  float _dFilterHz = 25.0f;

  // Output limits (kept as outMax/outMin as requested)
  float _outMax = 1e9f;
  float _outMin = -1e9f;

  // Anti-windup back-calculation gain
  float _awGain = 0.5f;
};

#endif