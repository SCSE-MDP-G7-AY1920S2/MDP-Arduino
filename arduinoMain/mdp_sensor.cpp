#include "mdp_sensor.h"

#include <Arduino.h>
#include "sharp_ir.h"

// Model Number for IR sensors.
#define SRmodel 1080
#define LRmodel 20150

// Sensor pins.
#define s0 A0  // front right
#define s1 A1  // front middle
#define s2 A2  // front left
#define s3 A3  // left long range
#define s4 A4  // right back
#define s5 A5  // right front

namespace {
// Maximum distance for Grid 1 and 2.
const int kFrontBorderGrid1 = 17;
const int kFrontBorderGrid2 = 27;
const int kRightBorderGrid1 = 16;
const int kRightFrontBorderGrid2 = 26;
const int kRightBackBorderGrid2 = 23;

const int kLRMax = 64;
const int kLROffset = 16;

// short IR sensor
// c - calibrated
ZSharpIR sr0c(s0, SRmodel, /*m=*/5.8027e+03, /*b=*/5.7073, /*k=*/3.2186);
ZSharpIR sr1c(s1, SRmodel, /*m=*/7.0216e+03, /*b=*/36.6182, /*k=*/4.5791);
ZSharpIR sr2c(s2, SRmodel, /*m=*/5.8207e+03, /*b=*/10.7342, /*k=*/3.1259);
ZSharpIR sr4c(s4, SRmodel, /*m=*/6.5677e+03, /*b=*/18.9407, /*k=*/3.7055);
ZSharpIR sr5c(s5, SRmodel, /*m=*/7.5070e+03, /*b=*/41.9073, /*k=*/5.0008);

// long IR sensor
ZSharpIR lr3c(s3, LRmodel, /*m=*/4.8193e+04, /*b=*/250.7146, /*k=*/46.2362);

// return distance from sensors (grids).
int getFrontDistance(ZSharpIR& sensor) {
  int dist = sensor.distance();
  if (dist < kFrontBorderGrid1)
    return 1;
  else if (dist < kFrontBorderGrid2)
    return 2;
  return -1;
}

int getRightDistance(ZSharpIR& sensor) {
  int dist = sensor.distance();
  bool isRightBack = sensor.getIrPin() == s4;
  if (dist < kRightBorderGrid1)
    return 1;
  else if ((!isRightBack && dist < kRightFrontBorderGrid2)
          || (isRightBack && dist < kRightBackBorderGrid2))
    return 2;
  return -1;
}

int getLeftDistance(ZSharpIR& sensor) {
  int dist = sensor.distance();
  if (dist < kLRMax) {
    int grid = (dist - kLROffset) / 10 + 1;
    return (grid > 0) ? grid : 1;
  }
  return -1;
}

// return distance from sensors (cm).
int getDistanceRaw(ZSharpIR& sensor) { return sensor.distance(); }
}  // namespace

void setupSensorsCalibration() {
}

int getFrontRight() { return getFrontDistance(sr0c); }

int getFrontRightRaw() { return getDistanceRaw(sr0c); }

int getFrontMiddle() { return getFrontDistance(sr1c); }

int getFrontMiddleRaw() { return getDistanceRaw(sr1c); }

int getFrontLeft() { return getFrontDistance(sr2c); }

int getFrontLeftRaw() { return getDistanceRaw(sr2c); }

int getLeft() { return getLeftDistance(lr3c); }

int getLeftRaw() { return getDistanceRaw(lr3c); }

int getRightBack() { return getRightDistance(sr4c); }

int getRightBackRaw() { return getDistanceRaw(sr4c); }

int getRightFront() { return getRightDistance(sr5c); }

int getRightFrontRaw() { return getDistanceRaw(sr5c); }

void calibrateRaw() {
  Serial.print("rf:");
  Serial.print(getRightFront());
  Serial.print(" rb:");
  Serial.print(getRightBack());
  Serial.print("\n");

  Serial.print("rf:");
  Serial.print(getRightFrontRaw());
  Serial.print(" rb:");
  Serial.print(getRightBackRaw());
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("\n");
  delay(1500);
}

// calibrate sensors
// calibration table of 20 value,
void calibrateSensors() {
  
}
