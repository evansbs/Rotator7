#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <Wire.h>

struct IMUData {
  float ax, ay, az; // Acceleration in g
  float mx, my, mz; // Magnetic field raw units / relative values
};

bool initIMU();
bool readIMU(IMUData &data);

#endif
