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
constexpr int kFrontBorderGrid1 = 17;
constexpr int kFrontBorderGrid2 = 27;
constexpr int kRightBorderGrid1 = 16;
constexpr int kRightFrontBorderGrid2 = 26;
constexpr int kRightBackBorderGrid2 = 25;

constexpr int kLRMax = 64;
constexpr int kLROffset = 16;

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
int getFrontDistance(const ZSharpIR& sensor) {
  int dist = sensor.distance();
  if (dist < kFrontBorderGrid1)
    return 1;
  else if (dist < kFrontBorderGrid2)
    return 2;
  return -1;
}

int getRightDistance(const ZSharpIR& sensor) {
  int dist = sensor.distance();
  bool isRightBack = sensor.getIrPin() == s4;
  if (dist < kRightBorderGrid1)
    return 1;
  else if ((!isRightBack && dist < kRightFrontBorderGrid2) ||
           (isRightBack && dist < kRightBackBorderGrid2))
    return 2;
  return -1;
}

int getLeftDistance(const ZSharpIR& sensor) {
  int dist = sensor.distance();
  if (dist < kLRMax) {
    int grid = (dist - kLROffset) / 10 + 1;
    return (grid > 0) ? grid : 1;
  }
  return -1;
}

// return distance from sensors (cm).
int getDistanceRaw(const ZSharpIR& sensor) { return sensor.distance(); }
}  // namespace

<<<<<<< HEAD
void setupSensorsCalibration() {}
=======
void setupSensorsCalibration() {
  int tablesr0c[] = {3,   457, 617, 618, 520, 440, 374, 327, 293, 263,
                     244, 224, 209, 192, 178, 170, 159, 151, 147, 0};
  int tablesr1c[] = {2,   355, 616, 618, 536, 452, 384, 337, 297, 271,
                     249, 231, 206, 189, 174, 160, 148, 138, 128, 0};
  int tablesr2c[] = {3,   346, 620, 620, 524, 439, 371, 324, 292, 263,
                     240, 218, 202, 186, 175, 167, 160, 146, 142, 0};

  int tablesr3c[] = {2,   374, 447, 563, 525, 489, 447, 408, 371, 330,
                     300, 274, 251, 233, 213, 201, 189, 182, 166, 0};
  // 236, 221, 205, 189, 178, 171, 0};

  int tablesr4c[] = {3,   241, 576, 620, 561, 465, 397, 347, 302, 273,
                     250, 226, 207, 191, 176, 165, 150, 143, 130, 0};

  int tablesr5c[] = {3,   346, 616, 622, 564, 470, 398, 347, 306, 277,
                     253, 230, 215, 198, 188, 176, 164, 157, 145, 0};

  sr0c.ApplyCalibration(tablesr0c);
  sr1c.ApplyCalibration(tablesr1c);
  sr2c.ApplyCalibration(tablesr2c);
  sr3c.ApplyCalibration(tablesr3c);
  sr4c.ApplyCalibration(tablesr4c);
  sr5c.ApplyCalibration(tablesr5c);
}
>>>>>>> master

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
  Serial.print("fm:");
  Serial.print(getFrontMiddle());
  Serial.print(" l:");
  Serial.println(getLeft());

  Serial.print("fmRaw:");
  Serial.print(getFrontMiddleRaw());
  Serial.print(" lRaw:");
  Serial.println(getLeftRaw());
  Serial.println();
  delay(1500);
}

// calibrate sensors
// calibration table of 20 value,
void calibrateSensors() {}
