#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "timer.h"
#include "sensor_imu.h"
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

enum Modes { tracking, monitoring, demonstrating, calibrating, debugging, pausing, faulted };

float az;
float el;
String line;
float azSet;
float elSet;
float azLast;
float elLast;
float azWindup;
float azOffset;
bool windup;
float azSpeed;
float elSpeed;
float azError;
float elError;
float azInc;
float elInc;
Modes mode;
bool imuOk = false;

Mot azMot(MotorType, azAlpha, azGain, azFwdPin, azRevPin);
SensorIMU imu;
Timer t1(100);

void save() { EEPROM.put(0, imu.cal); }
void restore() { EEPROM.get(0, imu.cal); }

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
  SerialPort.print("DBG ");
  SerialPort.print(imu.mx); SerialPort.print(",");
  SerialPort.print(imu.my); SerialPort.print(",");
  SerialPort.print(imu.mz); SerialPort.print(",");
  SerialPort.print(imu.ax); SerialPort.print(",");
  SerialPort.print(imu.ay); SerialPort.print(",");
  SerialPort.println(imu.az);
}

void printCal(void) {
  SerialPort.print("CAL ");
  SerialPort.print(imu.cal.md, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.me.i, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.me.j, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.me.k, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.ge.i, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.ge.j, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.ge.k, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.ms.i, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.ms.j, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.ms.k, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.gs.i, 1); SerialPort.print(",");
  SerialPort.print(imu.cal.gs.j, 1); SerialPort.print(",");
  SerialPort.println(imu.cal.gs.k, 1);
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
  SerialPort.print((az < 0) ? (az + 360) : az, 1);
  SerialPort.print("\n");
}

void printEl() {
  SerialPort.print("EL");
  SerialPort.print(el, 1);
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
  azSet = 0.0;
  elSet = 0.0;
  line = "";
  azLast = 0.0;
  elLast = 0.0;
  azWindup = 0.0;
  azOffset = 0.0;
  azSpeed = 0.0;
  elSpeed = 0.0;
  windup = false;
  azError = 0.0;
  elError = 0.0;
  azInc = 0.05;
  elInc = 0.05;
  t1.reset(100);

  if (getCal && imuOk) {
    restore();
    printCal();
  }

  if (imuOk) {
    imu.calStart();
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

void getWindup(bool *windup, float *azWindup, float *azOffset, float *azLast, float *elLast, float az, float elSet) {
  float azDiff = az - *azLast;
  if (azDiff < -180) *azOffset += 360;
  if (azDiff > 180) *azOffset -= 360;
  *azLast = az;
  *azWindup = az + *azOffset;
  if (abs(*azWindup) > WINDUP_LIMIT) *windup = true;
  *elLast = elSet;
}

void calibrate() {
  if (!imuOk) {
    enterFault("Calibration blocked: IMU not ready");
    return;
  }
  bool changed = imu.calibrate();
  if (changed) {
    digitalWrite(spkPin, HIGH);
    printCal();
  } else {
    digitalWrite(spkPin, LOW);
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
    *azError = constrain(azWindup, -180, 180);
    if (abs(*azError) < 175) *windup = false;
  } else {
    *azError = diffAngle(az, *azSet);
  }
  *elError = diffAngle(el, elSet);
}

void processPosition() {
  if (!imu.read()) {
    enterFault("IMU read failed");
    return;
  }

  if (!imuOk) {
    enterFault("IMU read invalid");
    return;
  }

  imu.getAzEl();
  az = imu.azimuth;
  el = imu.elevation;
  getWindup(&windup, &azWindup, &azOffset, &azLast, &elLast, az, elSet);
  if (mode == demonstrating) getAzElDemo(&azSet, &elSet, &azInc, &elInc);
  getAzElError(&azError, &elError, &windup, &azSet, elSet, az, el);

  switch (mode) {
    case debugging:
      printDebug();
      break;
    case calibrating:
      calibrate();
      break;
    case monitoring:
      printMon(az, el, azSet, elSet, azWindup, azError, elError);
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

  azMot.drive(azError);

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
      imu.cal.md = param.toFloat();
      SerialPort.print("ACK MagDecl=");
      SerialPort.println(imu.cal.md, 2);
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
        azSet = param.toFloat();
        param = cmd.substring(firstSpace + 1);
        elSet = param.toFloat();
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

  imuOk = imu.begin();
  if (!imuOk) {
    SerialPort.println("ERR IMU init failed");
    mode = faulted;
  } else {
    SerialPort.println("BOOT IMU init complete");
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
