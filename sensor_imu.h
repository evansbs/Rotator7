#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <Wire.h>

struct IMUData {
  float ax, ay, az;
  float mx, my, mz;
};

struct IMUVector3 {
  float i;
  float j;
  float k;
};

struct IMUCalibration {
  float md;      // magnetic declination in degrees
  IMUVector3 me; // magnetometer offset
  IMUVector3 ge; // gyro offset (unused on accel+mag hardware)
  IMUVector3 ms; // magnetometer scale
  IMUVector3 gs; // gyro scale (unused on accel+mag hardware)
};

class SensorIMU {
public:
  SensorIMU();

  bool begin();
  bool read();
  void getAzEl();
  bool calibrate();
  void calStart();

  IMUCalibration cal;
  float ax, ay, az;
  float mx, my, mz;
  float azimuth;
  float elevation;
};

#endif
