#include "mdp_sensor.h"

#include <Arduino.h>
#include <ZSharpIR.h>

#include "cf_sharp_ir.h"

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
constexpr int kFrontBorderGrid1 = 17;
constexpr int kFrontBorderGrid2 = 27;
constexpr int kRightBorderGrid1 = 16;
constexpr int kRightFrontBorderGrid2 = 26;
constexpr int kRightBackBorderGrid2 = 25;

constexpr int kLRMax = 64;
constexpr int kLROffset = 9;

// short IR sensor
// c - calibrated
CFSharpIR sr0c(s0, SRmodel, /*m=*/5.8027e+03, /*b=*/5.7073, /*k=*/3.2186);
CFSharpIR sr1c(s1, SRmodel, /*m=*/7.0216e+03, /*b=*/36.6182, /*k=*/4.5791);
CFSharpIR sr2c(s2, SRmodel, /*m=*/5.8207e+03, /*b=*/10.7342, /*k=*/3.1259);
CFSharpIR sr4c(s4, SRmodel, /*m=*/6.5677e+03, /*b=*/18.9407, /*k=*/3.7055);
CFSharpIR sr5c(s5, SRmodel, /*m=*/7.5070e+03, /*b=*/41.9073, /*k=*/5.0008);

// long IR sensor
ZSharpIR sr3c(s3, LRmodel);

// return distance from sensors (grids).
int getFrontDistance(const CFSharpIR& sensor) {
  int dist = round(sensor.distance() / 10.0);
  if (dist < kFrontBorderGrid1)
    return 1;
  else if (dist < kFrontBorderGrid2)
    return 2;
  return -1;
}

int getRightDistance(const CFSharpIR& sensor) {
  int dist = round(sensor.distance() / 10.0);
  bool isRightBack = sensor.getIrPin() == s4;
  if (dist < kRightBorderGrid1)
    return 1;
  else if ((!isRightBack && dist < kRightFrontBorderGrid2) ||
           (isRightBack && dist < kRightBackBorderGrid2))
    return 2;
  return -1;
}

int getLeftDistance(ZSharpIR& sensor) {
  int dist = round(sensor.distance() / 10.0);
  if (dist < kLRMax) {
    int grid = (dist - kLROffset) / 10.0 + 1;
    return (grid > 0) ? grid : 1;
  }
  return -1;
}

// return distance from sensors (cm).
int getDistanceRaw(const CFSharpIR& sensor) { return sensor.distance(); }
}  // namespace

void setupSensorsCalibration() {
    int tablesr3c[] = {2,   374, 447, 563, 525, 489, 447, 408, 371, 330,
                     300, 274, 251, 233, 213, 201, 189, 182, 166, 0};
    sr3c.ApplyCalibration(tablesr3c);
}

int getFrontRight() { return getFrontDistance(sr0c); }

int getFrontRightRaw() { return getDistanceRaw(sr0c); }

int getFrontMiddle() { return getFrontDistance(sr1c); }

int getFrontMiddleRaw() { return getDistanceRaw(sr1c); }

int getFrontLeft() { return getFrontDistance(sr2c); }

int getFrontLeftRaw() { return getDistanceRaw(sr2c); }

int getLeft() { return getLeftDistance(sr3c); }

int getLeftRaw() { return sr3c.distance(); }

int getRightBack() { return getRightDistance(sr4c); }

int getRightBackRaw() { return getDistanceRaw(sr4c); }

int getRightFront() { return getRightDistance(sr5c); }

int getRightFrontRaw() { return getDistanceRaw(sr5c); }

void calibrateRaw() {
  Serial.print("rf:");
  Serial.print(getRightFrontRaw());
  Serial.print(" rb:");
  Serial.println(getRightBackRaw());

  Serial.print("frRaw:");
  Serial.print(getFrontRightRaw());
  Serial.print(" flRaw:");
  Serial.println(getFrontLeftRaw());
  Serial.println();
  delay(1500);
}

// calibrate sensors
// calibration table of 20 value,
void calibrateSensors() {}
