#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

class motorControl{

  private:
    const int ENA = 25;   // Speed (PWM)
    const int IN1 = 27;   // Direction
    const int IN2 = 26;   // Direction

    int clampInt(int x, int lo, int hi) {
      if (x < lo) return lo;
      if (x > hi) return hi;
      return x;
    }
  public:
    void setupMotor() {
      pinMode(ENA, OUTPUT);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);

      setMotorPower(0);
    }

    void setMotorPower(int power) {
      power = clampInt(power, -100, 100);

      int pwm = map(abs(power), 0, 100, 0, 255);

      if (power > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      } else if (power < 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW); // coast
      }

      analogWrite(ENA, pwm);
    }


};



#endif