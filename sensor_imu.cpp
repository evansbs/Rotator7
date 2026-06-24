#include "sensor_imu.h"

// LSM303DLHC-style addresses
static const uint8_t ACCEL_ADDR = 0x19;
static const uint8_t MAG_ADDR   = 0x1E;

// Accelerometer registers
static const uint8_t ACC_CTRL_REG1_A = 0x20;
static const uint8_t ACC_CTRL_REG4_A = 0x23;
static const uint8_t ACC_OUT_X_L_A   = 0x28;

// Magnetometer registers
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
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t n = Wire.requestFrom(addr, len);
  if (n != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool initIMU() {
  Wire.begin();
  Wire.setClock(100000);

  // Accelerometer: 50 Hz, all axes enabled
  if (!writeReg(ACCEL_ADDR, ACC_CTRL_REG1_A, 0x27)) {
    return false;
  }

  // Accelerometer: +/-2g
  if (!writeReg(ACCEL_ADDR, ACC_CTRL_REG4_A, 0x00)) {
    return false;
  }

  // Verify magnetometer identity bytes
  uint8_t id[3] = {0, 0, 0};
  if (!readRegs(MAG_ADDR, MAG_IRA_REG_M, id, 3)) {
    return false;
  }

  if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) {
    return false;
  }

  // Magnetometer continuous-conversion mode
  if (!writeReg(MAG_ADDR, MAG_MR_REG_M, 0x00)) {
    return false;
  }

  // Magnetometer gain: +/-1.3 gauss
  if (!writeReg(MAG_ADDR, MAG_CRB_REG_M, 0x20)) {
    return false;
  }

  // Magnetometer data rate: 15 Hz (CRA_REG_M bits 4:2 = 0x04)
  if (!writeReg(MAG_ADDR, MAG_CRA_REG_M, 0x14)) {
    return false;
  }

  return true;
}

bool readIMU(IMUData &data) {
  // Accelerometer raw read: low byte first, auto-increment
  uint8_t acc[6];
  if (!readRegs(ACCEL_ADDR, ACC_OUT_X_L_A | 0x80, acc, 6)) {
    return false;
  }

  int16_t raw_ax = (int16_t)((uint16_t)acc[1] << 8 | acc[0]) >> 4;
  int16_t raw_ay = (int16_t)((uint16_t)acc[3] << 8 | acc[2]) >> 4;
  int16_t raw_az = (int16_t)((uint16_t)acc[5] << 8 | acc[4]) >> 4;

  // +/-2g => 16384 LSB/g
  data.ax = (float)raw_ax / 16384.0f;
  data.ay = (float)raw_ay / 16384.0f;
  data.az = (float)raw_az / 16384.0f;

  // Magnetometer raw read: high byte first
  uint8_t mag[6];
  if (!readRegs(MAG_ADDR, MAG_OUT_X_H_M, mag, 6)) {
    return false;
  }

  int16_t raw_mx = (int16_t)((uint16_t)mag[0] << 8 | mag[1]);
  int16_t raw_mz = (int16_t)((uint16_t)mag[2] << 8 | mag[3]);
  int16_t raw_my = (int16_t)((uint16_t)mag[4] << 8 | mag[5]);

  // Temporary reuse of gx/gy/gz for magnetometer vector
  data.gx = (float)raw_mx;
  data.gy = (float)raw_my;
  data.gz = (float)raw_mz;

  return true;
}
