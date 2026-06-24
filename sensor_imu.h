#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <Wire.h>

struct IMUData {
  // LSM303-style accelerometer data, expressed in g.
  float ax, ay, az;

  // LSM303-style magnetometer data.
  // These are left as raw / relative values so the caller can apply
  // calibration, scaling, or heading math as needed.
  float mx, my, mz;
};

// Initialize the LSM303DLHC-style accelerometer + magnetometer pair.
bool initIMU();

// Read one combined accelerometer + magnetometer sample into `data`.
bool readIMU(IMUData &data);

#endif
