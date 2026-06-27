#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "timer.h"
#include "NanoIMU.h"
#include "mot.h"

const int MotorType = FWDREV;
#define SerialPort Serial
#define WINDUP_LIMIT 360

const int azFwdPin = 5;
const int azRevPin = 6;
const int azBrkPin = 7;
const int elBrkPin = 8;
const int elFwdPin = 9;
const int elRevPin = 10;
const int spkPin = 11;
const int gndPin = 12;

const int azGain = 25;
const float azAlpha = 0.5;

const bool HARD_DISABLE_EL = false;
const int EEPROM_BASE = 0;

enum Modes { tracking, monitoring, demonstrating, calibrating, debugging, pausing, faulted };

float azimuthDeg;
float elevationDeg;
float azimuthSetpointDeg;
float elevationSetpointDeg;
float lastAzimuthDeg;
float lastElevationDeg;
float azimuthWindupDeg;
float azimuthOffsetDeg;
bool windup;
float azSpeed;
float elSpeed;
float azimuthErrorDeg;
float elevationErrorDeg;
float azimuthIncrementDeg;
float elevationIncrementDeg;

String line;
Modes mode;
bool imuOk = false;
bool calDirty = false;

Mot azMot(MotorType, azAlpha, azGain, azFwdPin, azRevPin);
NanoIMU imu;
Timer t1(100);

void save() {
  if (imu.saveSettingsToEEPROM(EEPROM_BASE)) {
    calDirty = false;
  }
}

void restore() {
  imu.loadSettingsFromEEPROM(EEPROM_BASE);
}

void forceElSafe() {
  pinMode(elFwdPin, OUTPUT);
  pinMode(elRevPin, OUTPUT);
  pinMode(elBrkPin, OUTPUT);
  digitalWrite(elFwdPin, LOW);
  digitalWrite(elRevPin, LOW);
  digitalWrite(elBrkPin, LOW);
}

void releaseEl() {
  pinMode(elFwdPin, OUTPUT);
  pinMode(elRevPin, OUTPUT);
  pinMode(elBrkPin, OUTPUT);
}

void printDebug(void) {
  NanoIMU::Vec3f accOff = imu.getAccelOffset();
  NanoIMU::Vec3f accScale = imu.getAccelScale();
  NanoIMU::Vec3f magOff = imu.getMagOffset();
  NanoIMU::Vec3f magScale = imu.getMagScale();

  SerialPort.print("DBG ");
  SerialPort.print(imu.getDeclinationDeg(), 2); SerialPort.print(",");
  SerialPort.print(accOff.x, 3); SerialPort.print(",");
  SerialPort.print(accOff.y, 3); SerialPort.print(",");
  SerialPort.print(accOff.z, 3); SerialPort.print(",");
  SerialPort.print(accScale.x, 3); SerialPort.print(",");
  SerialPort.print(accScale.y, 3); SerialPort.print(",");
  SerialPort.print(accScale.z, 3); SerialPort.print(",");
  SerialPort.print(magOff.x, 3); SerialPort.print(",");
  SerialPort.print(magOff.y, 3); SerialPort.print(",");
  SerialPort.print(magOff.z, 3); SerialPort.print(",");
  SerialPort.print(magScale.x, 3); SerialPort.print(",");
  SerialPort.print(magScale.y, 3); SerialPort.print(",");
  SerialPort.println(magScale.z, 3);
}

void printCal(void) {
  NanoIMU::Vec3f accOff = imu.getAccelOffset();
  NanoIMU::Vec3f accScale = imu.getAccelScale();
  NanoIMU::Vec3f magOff = imu.getMagOffset();
  NanoIMU::Vec3f magScale = imu.getMagScale();

  SerialPort.print("CAL ");
  SerialPort.print(imu.getDeclinationDeg(), 1); SerialPort.print(",");
  SerialPort.print(accOff.x, 1); SerialPort.print(",");
  SerialPort.print(accOff.y, 1); SerialPort.print(",");
  SerialPort.print(accOff.z, 1); SerialPort.print(",");
  SerialPort.print(0.0f, 1); SerialPort.print(",");
  SerialPort.print(0.0f, 1); SerialPort.print(",");
  SerialPort.print(0.0f, 1); SerialPort.print(",");
  SerialPort.print(magScale.x, 1); SerialPort.print(",");
  SerialPort.print(magScale.y, 1); SerialPort.print(",");
  SerialPort.print(magScale.z, 1); SerialPort.print(",");
  SerialPort.print(1.0f, 1); SerialPort.print(",");
  SerialPort.print(1.0f, 1); SerialPort.print(",");
  SerialPort.println(1.0f, 1);
}

void printMon(float az, float el, float azSet, float elSet, float azWindup, float azError, float elError) {
  SerialPort.print("MON ");
  SerialPort.print(az, 0); SerialPort.print(",");
  SerialPort.print(el, 0); SerialPort.print(",");
  SerialPort.print(azSet, 0); SerialPort.print(",");
  SerialPort.print(elSet, 0); SerialPort.print(",");
  SerialPort.print(azWindup, 0); SerialPort.print(",");
  SerialPort.print(windup); SerialPort.print(",");
  SerialPort.print(azError, 0); SerialPort.print(",");
  SerialPort.println(elError, 0);
}

void printAz() {
  SerialPort.print("AZ");
  SerialPort.print((azimuthDeg < 0) ? (azimuthDeg + 360) : azimuthDeg, 1);
  SerialPort.print("\n");
}

void printEl() {
  SerialPort.print("EL");
  SerialPort.print(elevationDeg, 1);
  SerialPort.print("\n");
}

void enterFault(const char *msg) {
  mode = faulted;
  imuOk = false;
  azMot.halt();
  SerialPort.print("FAULT ");
  SerialPort.println(msg);
}

void reset(bool getCal) {
  azimuthSetpointDeg = 0.0;
  elevationSetpointDeg = 0.0;
  line = "";
  lastAzimuthDeg = 0.0;
  lastElevationDeg = 0.0;
  azimuthWindupDeg = 0.0;
  azimuthOffsetDeg = 0.0;
  azSpeed = 0.0;
  elSpeed = 0.0;
  windup = false;
  azimuthErrorDeg = 0.0;
  elevationErrorDeg = 0.0;
  azimuthIncrementDeg = 0.05;
  elevationIncrementDeg = 0.05;
  t1.reset(100);

  if (getCal && imuOk) {
    restore();
    printCal();
  }

  if (imuOk) {
    if (mode == faulted) mode = pausing;
  } else {
    enterFault("IMU not ready");
  }
}

float diffAngle(float a, float b) {
  float diff = a - b;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return diff;
}

void getWindup(bool *windup, float *azWindupDeg, float *azOffsetDeg, float *lastAzimuthDeg, float *lastElevationDeg, float az, float elSet) {
  float azDiff = az - *lastAzimuthDeg;
  if (azDiff < -180) *azOffsetDeg += 360;
  if (azDiff > 180) *azOffsetDeg -= 360;
  *lastAzimuthDeg = az;
  *azWindupDeg = az + *azOffsetDeg;
  if (abs(*azWindupDeg) > WINDUP_LIMIT) *windup = true;
  *lastElevationDeg = elSet;
}

void calibrate() {
  if (!imuOk) {
    enterFault("Calibration blocked: IMU not ready");
    return;
  }

  NanoIMU::Vec3f magOffset, magScale;
  SerialPort.println("CAL rotate sensor slowly...");
  if (imu.calibrateMagMinMax(180, 20, magOffset, magScale)) {
    imu.setMagCalibration(magOffset, magScale);
    calDirty = true;
    digitalWrite(spkPin, HIGH);
    printCal();
    save();
    digitalWrite(spkPin, LOW);
    SerialPort.println("ACK cal ok");
  } else {
    digitalWrite(spkPin, LOW);
    SerialPort.println("ERR cal failed");
  }
}

void getAzElDemo(float *azSet, float *elSet, float *azInc, float *elInc) {
  if (*azSet > 180.0) *azInc = -*azInc;
  if (*azSet < -180.0) *azInc = -*azInc;
  if (*elSet > 90.0) *elInc = -*elInc;
  if (*elSet < 0.0) *elInc = -*elInc;
  *azSet += *azInc;
  *elSet += *elInc;
  SerialPort.print(*azSet, 0); SerialPort.print(",");
  SerialPort.println(*elSet, 0);
}

void getAzElError(float *azError, float *elError, bool *windup, float *azSet, float elSet, float az, float el) {
  if (*windup) {
    *azError = constrain(azimuthWindupDeg, -180, 180);
    if (abs(*azError) < 175) *windup = false;
  } else {
    *azError = diffAngle(az, *azSet);
  }
  *elError = diffAngle(el, elSet);
}

void processPosition() {
  NanoIMU::Vec3f acc;
  if (!imu.readAccel(acc)) {
    enterFault("IMU read failed");
    return;
  }

  NanoIMU::Vec3f mag;
  if (!imu.readMag(mag)) {
    enterFault("IMU mag read failed");
    return;
  }

  if (!imuOk) {
    enterFault("IMU read invalid");
    return;
  }

  float headingDeg;
  if (!imu.getHeadingDeg(headingDeg, true)) {
    enterFault("IMU heading failed");
    return;
  }

  azimuthDeg = headingDeg;
  elevationDeg = atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180.0f / PI;
  if (elevationDeg < 0) elevationDeg = -elevationDeg;

  getWindup(&windup, &azimuthWindupDeg, &azimuthOffsetDeg, &lastAzimuthDeg, &lastElevationDeg, azimuthDeg, elevationSetpointDeg);

  if (mode == demonstrating) {
    getAzElDemo(&azimuthSetpointDeg, &elevationSetpointDeg, &azimuthIncrementDeg, &elevationIncrementDeg);
  }

  getAzElError(&azimuthErrorDeg, &elevationErrorDeg, &windup, &azimuthSetpointDeg, elevationSetpointDeg, azimuthDeg, elevationDeg);

  switch (mode) {
    case debugging:
      printDebug();
      break;
    case calibrating:
      calibrate();
      break;
    case monitoring:
      printMon(azimuthDeg, elevationDeg, azimuthSetpointDeg, elevationSetpointDeg, azimuthWindupDeg, azimuthErrorDeg, elevationErrorDeg);
      break;
    case pausing:
    case faulted:
      break;
    default:
      break;
  }
}

void processMotors() {
  if (mode == faulted || mode == pausing || !imuOk) {
    azMot.halt();
    if (HARD_DISABLE_EL) forceElSafe();
    return;
  }

  azMot.drive(azimuthErrorDeg);

  if (HARD_DISABLE_EL) {
    forceElSafe();
  } else {
    releaseEl();
  }
}

void processUserCommands(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char command = cmd.charAt(0);
  String param;
  int firstSpace;

  switch (command) {
    case 'r':
      SerialPort.println("ACK reset");
      reset(imuOk);
      break;

    case 'b':
      SerialPort.println("ACK debug");
      mode = debugging;
      t1.reset(100);
      break;

    case 'm':
      SerialPort.println("ACK monitor");
      if (imuOk) mode = monitoring;
      else enterFault("Cannot monitor: IMU not ready");
      t1.reset(100);
      break;

    case 'c':
      if (!imuOk) {
        SerialPort.println("ERR IMU not ready");
        enterFault("Calibration refused");
        break;
      }
      SerialPort.println("ACK calibrate");
      reset(false);
      mode = calibrating;
      t1.reset(50);
      break;

    case 'a':
      SerialPort.println("ACK abort");
      mode = pausing;
      t1.reset(100);
      if (imuOk) reset(true);
      break;

    case 'e':
      param = cmd.substring(1);
      imu.setDeclinationDeg(param.toFloat());
      calDirty = true;
      SerialPort.print("ACK MagDecl=");
      SerialPort.println(imu.getDeclinationDeg(), 2);
      break;

    case 's':
      save();
      if (imuOk) reset(true);
      SerialPort.println("ACK save");
      break;

    case 'd':
      if (!imuOk) {
        SerialPort.println("ERR IMU not ready");
        enterFault("Demo refused");
        break;
      }
      SerialPort.println("ACK demo");
      t1.reset(50);
      mode = demonstrating;
      break;

    case 'h':
      SerialPort.println("Commands:");
      SerialPort.println("az el -(0..360 0..90)");
      SerialPort.println("r -Reset");
      SerialPort.println("eNN.N -MagDecl");
      SerialPort.println("c -Calibrate");
      SerialPort.println("s -Save");
      SerialPort.println("a -Abort");
      SerialPort.println("d -Demo");
      SerialPort.println("b -Debug");
      SerialPort.println("m -Monitor");
      SerialPort.println("p -Pause");
      SerialPort.println("EL drive remains enabled in this build unless faulted.");
      SerialPort.println("IMU uses NanoIMU accel + mag; no gyro is present.");
      break;

    case 'p':
      if (mode == pausing) {
        if (imuOk) {
          mode = tracking;
          SerialPort.println("ACK resume");
        } else {
          SerialPort.println("ERR IMU not ready");
          enterFault("Resume refused");
        }
      } else {
        mode = pausing;
        SerialPort.println("ACK pause");
      }
      break;

    case 'J':
      SerialPort.println("ACK JOG ignored");
      break;

    default:
      firstSpace = cmd.indexOf(' ');
      if (firstSpace > 0) {
        param = cmd.substring(0, firstSpace);
        azimuthSetpointDeg = param.toFloat();
        param = cmd.substring(firstSpace + 1);
        elevationSetpointDeg = param.toFloat();
        SerialPort.println("ACK setpoint");
      } else {
        SerialPort.print("ERR Unknown command: ");
        SerialPort.println(cmd);
      }
      break;
  }
}

void processEasycommCommands(String cmd) {
  if (cmd == "AZ") {
    printAz();
  } else if (cmd == "EL") {
    printEl();
  }
}

void processCommands() {
  while (SerialPort.available() > 0) {
    char ch = SerialPort.read();
    switch (ch) {
      case 13:
        processUserCommands(line);
        line = "";
        break;
      case 10:
        processEasycommCommands(line);
        line = "";
        break;
      default:
        line += ch;
        if (line.length() > 120) line = "";
        break;
    }
  }
}

void setup() {
  pinMode(spkPin, OUTPUT);
  pinMode(gndPin, OUTPUT);
  digitalWrite(gndPin, LOW);

  pinMode(azBrkPin, OUTPUT);
  digitalWrite(azBrkPin, LOW);
  pinMode(azFwdPin, OUTPUT);
  pinMode(azRevPin, OUTPUT);
  digitalWrite(azFwdPin, LOW);
  digitalWrite(azRevPin, LOW);

  if (HARD_DISABLE_EL) forceElSafe();
  else releaseEl();

  SerialPort.begin(9600);
  delay(200);
  SerialPort.println("BOOT start");

  Wire.begin();
  Wire.setClock(100000);

  imuOk = imu.begin();
  if (!imuOk) {
    SerialPort.println("ERR IMU init failed");
    mode = faulted;
  } else {
    restore();
    SerialPort.println("BOOT IMU init complete");
    printCal();
    mode = tracking;
  }

  reset(true);
  SerialPort.println(imuOk ? "BOOT ready" : "BOOT faulted");
}

void loop() {
  processCommands();
  t1.execute(&processPosition);
  processMotors();
}
