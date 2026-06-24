#include "sensor_imu.h"
#include <math.h>

static const uint8_t ACCEL_ADDR = 0x19;
static const uint8_t MAG_ADDR   = 0x1E;
static const uint8_t ACC_CTRL_REG1_A = 0x20;
static const uint8_t ACC_CTRL_REG4_A = 0x23;
static const uint8_t ACC_OUT_X_L_A   = 0x28;
static const uint8_t MAG_CRA_REG_M   = 0x00;
static const uint8_t MAG_CRB_REG_M   = 0x01;
static const uint8_t MAG_MR_REG_M    = 0x02;
static const uint8_t MAG_OUT_X_H_M   = 0x03;
static const uint8_t MAG_IRA_REG_M   = 0x0A;

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom(addr, len);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool probeAddr(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

SensorIMU::SensorIMU()
  : ax(0), ay(0), az(0), mx(0), my(0), mz(0), azimuth(0), elevation(0) {
  cal.md = 0.0f;
  cal.me = {0, 0, 0};
  cal.ge = {0, 0, 0};
  cal.ms = {1, 1, 1};
  cal.gs = {1, 1, 1};
}

bool SensorIMU::begin() {
  Wire.begin();
  Wire.setClock(100000);

  Serial.print("IMU PROBE ACC ");
  Serial.println(probeAddr(ACCEL_ADDR) ? "OK" : "FAIL");
  Serial.print("IMU PROBE MAG ");
  Serial.println(probeAddr(MAG_ADDR) ? "OK" : "FAIL");

  if (!writeReg(ACCEL_ADDR, ACC_CTRL_REG1_A, 0x27)) return false;
  if (!writeReg(ACCEL_ADDR, ACC_CTRL_REG4_A, 0x00)) return false;

  uint8_t id[3] = {0, 0, 0};
  if (!readRegs(MAG_ADDR, MAG_IRA_REG_M, id, 3)) return false;
  Serial.print("IMU MAG ID ");
  Serial.print(id[0], HEX); Serial.print(" ");
  Serial.print(id[1], HEX); Serial.print(" ");
  Serial.println(id[2], HEX);
  if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) return false;

  if (!writeReg(MAG_ADDR, MAG_MR_REG_M, 0x00)) return false;
  if (!writeReg(MAG_ADDR, MAG_CRB_REG_M, 0x20)) return false;
  if (!writeReg(MAG_ADDR, MAG_CRA_REG_M, 0x14)) return false;

  Serial.println("IMU init complete");
  return true;
}

bool SensorIMU::read() {
  uint8_t acc[6];
  if (!readRegs(ACCEL_ADDR, ACC_OUT_X_L_A | 0x80, acc, 6)) {
    Serial.println("IMU ACC read failed");
    return false;
  }

  int16_t raw_ax = (int16_t)((uint16_t)acc[1] << 8 | acc[0]) >> 4;
  int16_t raw_ay = (int16_t)((uint16_t)acc[3] << 8 | acc[2]) >> 4;
  int16_t raw_az = (int16_t)((uint16_t)acc[5] << 8 | acc[4]) >> 4;

  ax = (float)raw_ax / 16384.0f;
  ay = (float)raw_ay / 16384.0f;
  az = (float)raw_az / 16384.0f;

  uint8_t mag[6];
  if (!readRegs(MAG_ADDR, MAG_OUT_X_H_M, mag, 6)) {
    Serial.println("IMU MAG read failed");
    return false;
  }

  int16_t raw_mx = (int16_t)((uint16_t)mag[0] << 8 | mag[1]);
  int16_t raw_mz = (int16_t)((uint16_t)mag[2] << 8 | mag[3]);
  int16_t raw_my = (int16_t)((uint16_t)mag[4] << 8 | mag[5]);

  mx = (float)raw_mx;
  my = (float)raw_my;
  mz = (float)raw_mz;

  return true;
}

void SensorIMU::getAzEl() {
  // Adafruit 10DOF-style tilt compensation:
  // 1) derive roll/pitch from accelerometer
  // 2) rotate magnetometer into the horizontal plane
  // 3) compute heading from the compensated magnetic vector

  const float PI_F = 3.14159265f;

  float roll = atan2(ay, az);
  float pitch = atan(-ax / (ay * sin(roll) + az * cos(roll)));

  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt-compensated magnetic field components
  float magXh = mx * cosPitch + mz * sinPitch;
  float magYh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

  float headingDeg = atan2(magYh, magXh) * 180.0f / PI_F;
  headingDeg += cal.md;
  if (headingDeg < 0) headingDeg += 360.0f;
  if (headingDeg >= 360.0f) headingDeg -= 360.0f;

  azimuth = headingDeg;

  // Elevation: use the tilt angle from the accelerometer as a simple sky/tilt estimate.
  // This keeps the existing az/el sketch structure intact.
  elevation = pitch * 180.0f / PI_F;
  if (elevation < 0) elevation = -elevation;
}

bool SensorIMU::calibrate() { return false; }
void SensorIMU::calStart() {}
