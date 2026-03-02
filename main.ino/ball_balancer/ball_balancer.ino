#include "mpu_interface.h"
#include "pid.h"
#include "motor_control.h"

// Remove const so they can change
double kp = 12.0;
double ki = 0.0;
double kd = 2.0;

double output_max = 100;
double output_min = -100;

PID pid_control(0,0,0, output_max, output_min);  // initialize with zeros
mpuInterface mpu_IP;
motorControl motor;

double pitch;
double set_point = 0.0;
double control_output = 0.0;
uint32_t lastPrint = micros();
const uint32_t LOOP_US = 5000; // 5ms = 200Hz
uint32_t lastLoop = 0;

void waitForPIDInput() {
  Serial.println("Enter kp ki kd separated by spaces:");
  Serial.println("Example: 12.0 0.0 2.0");

  while (!Serial.available()) {
    // wait here (only happens once at startup)
  }

  kp = Serial.parseFloat();
  ki = Serial.parseFloat();
  kd = Serial.parseFloat();

  Serial.println("PID values updated:");
  Serial.print("kp: "); Serial.println(kp);
  Serial.print("ki: "); Serial.println(ki);
  Serial.print("kd: "); Serial.println(kd);

  // Update PID controller gains
  pid_control.setTunings(kp, ki, kd);

  // Clear buffer
  while (Serial.available()) Serial.read();
}

void setup() {
  Serial.begin(115200);

  mpu_IP.setup_mpu();
  motor.setupMotor();

  delay(2000);  // Give Serial Monitor time to open
  waitForPIDInput();  // Only runs once
}

void loop() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastLoop) < LOOP_US) return;

  float dt = (now - lastLoop) / 1000000.0f;
  lastLoop = now;

  pitch = mpu_IP.get_pitch();
  control_output = pid_control.compute(set_point, pitch, dt);
  motor.setMotorPower(control_output);

  if (now-lastPrint >= 50000){ //50 ms, 20 hz
    lastPrint = now;
    Serial.print(pitch,1);
    Serial.print(" "); Serial.println(control_output,1);

  }

}