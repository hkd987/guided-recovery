// Inertial and pressure sensors on the I2C bus.
#pragma once

#include <Arduino.h>

#include "gr/math.hpp"

enum class ImuType : uint8_t { None, Bmi088, Lsm6dso32 };

struct SensorStatus {
  ImuType imu = ImuType::None;
  uint8_t imuAddr = 0, gyroAddr = 0;
  bool baro = false;
  uint8_t baroAddr = 0;
  bool highG = false;
  uint32_t imuErrors = 0, baroErrors = 0;
};

extern SensorStatus g_sensors;

// Probe the bus and configure whatever is found. Safe to call again.
void sensorsBegin();

// Read accel (m/s^2, specific force) and gyro (rad/s) in the SENSOR frame.
bool imuRead(gr::Vec3& accel, gr::Vec3& gyro);
// Rotate a sensor-frame vector into the pod body frame (settings imu.map).
gr::Vec3 toBody(const gr::Vec3& v);

// Returns true when a new pressure sample is available.
bool baroRead(float& pressurePa, float& tempC);

// Optional ADXL375, m/s^2 in the sensor frame.
bool highGRead(gr::Vec3& accel);

// I2C scan for the self-test.
void i2cScan(Print& out);
