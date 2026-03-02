#ifndef MPU_INTERFACE_H
#define MPU_INTERFACE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

class mpuInterface {
private:
  Adafruit_MPU6050 mpu;

  // Reuse event structs
  sensors_event_t aEvt, gEvt, tEvt;

  // Timing / caching
  uint32_t lastUpdateMicros = 0;
  float lastPitch = 0.0f;
  float lastRoll  = 0.0f;

  // Target update rate (Hz)
  static constexpr uint16_t kRateHz = 200;
  static constexpr uint32_t kPeriodUs = 1000000UL / kRateHz;

public:
  bool setup_mpu() {
    Wire.begin(21, 22);
    Wire.setClock(1000000);

    if (!mpu.begin()) {
      Serial.println("MPU6050 not found");
      return false;
    }

    // Accel-only tilt prefers a bit of filtering to reduce jitter
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    lastUpdateMicros = micros();

    // Prime one read
    mpu.getEvent(&aEvt, &gEvt, &tEvt);
    return true;
  }

  // Accel-only pitch/roll in degrees (fast return to level, but noisy during motion)
  inline float get_pitch() {
    update_if_due();
    return lastPitch;
  }

  inline float get_roll() {
    update_if_due();
    return lastRoll;
  }

private:
  inline void update_if_due() {
    const uint32_t now = micros();
    if ((uint32_t)(now - lastUpdateMicros) < kPeriodUs) return;
    lastUpdateMicros += kPeriodUs;

    mpu.getEvent(&aEvt, &gEvt, &tEvt);

    const float ax = aEvt.acceleration.x; // m/s^2
    const float ay = aEvt.acceleration.y;
    const float az = aEvt.acceleration.z;

    // Standard tilt from gravity (degrees)
    // Pitch: rotation around Y (depends on your axis convention)
    const float pitchRad = atan2f(-ax, sqrtf(ay * ay + az * az));
    const float rollRad  = atan2f( ay, az );

    lastPitch = pitchRad * 180.0f / (float)M_PI;
    lastRoll  = rollRad  * 180.0f / (float)M_PI;
  }
};

#endif