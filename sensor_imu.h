#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <Wire.h>

struct IMUData {
  float ax, ay, az; // Acceleration in g
  float gx, gy, gz; // Temporary reuse for magnetometer vector
};

bool initIMU();
bool readIMU(IMUData &data);

#endif
