#include "mdp_sensor.h"

#include <Arduino.h>
#include <ZSharpIR.h>

// Model Number for IR sensors.
#define SRmodel 1080
#define LRmodel 20150

// Sensor pins.
#define s0 A0  // front right
#define s1 A1  // front middle
#define s2 A2  // front left
#define s3 A3  // left long range
#define s4 A4  // right bottom
#define s5 A5  // right top

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
ZSharpIR sr0c(s0, SRmodel);
ZSharpIR sr1c(s1, SRmodel);
ZSharpIR sr2c(s2, SRmodel);
ZSharpIR sr4c(s4, SRmodel);
ZSharpIR sr5c(s5, SRmodel);

// long IR sensor
ZSharpIR sr3c(s3, LRmodel);

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
  int tablesr0c[] = {3,   457, 617, 618, 520, 440, 374, 327, 293, 263,
                     244, 224, 209, 192, 178, 170, 159, 151, 147, 0};
  int tablesr1c[] = {2,   355, 616, 618, 536, 452, 384, 337, 297, 271,
                     249, 231, 206, 189, 174, 160, 148, 138, 128, 0};
  int tablesr2c[] = {3,   346, 620, 620, 524, 439, 371, 324, 292, 263,
                     240, 218, 202, 186, 175, 167, 160, 146, 142, 0};

  int tablesr3c[] = {2,   374, 447, 563, 525, 489, 447, 408, 371, 330,
                     300, 274, 251, 233, 213, 201, 189, 182, 166, 0};
                     //236, 221, 205, 189, 178, 171, 0};
                     
  int tablesr4c[] = {3,241,576,620,561,465,397,347,302,273,250,226,207,191,176,165,150,143,130,0};
                     
  int tablesr5c[] = {3,   346, 616, 622, 564, 470, 398, 347, 306, 277,
                     253, 230, 215, 198, 188, 176, 164, 157, 145, 0};

  sr0c.ApplyCalibration(tablesr0c);
  sr1c.ApplyCalibration(tablesr1c);
  sr2c.ApplyCalibration(tablesr2c);
  sr3c.ApplyCalibration(tablesr3c);
  sr4c.ApplyCalibration(tablesr4c);
  sr5c.ApplyCalibration(tablesr5c);
}

int getFrontRight() { return getFrontDistance(sr0c); }

int getFrontRightRaw() { return getDistanceRaw(sr0c); }

int getFrontMiddle() { return getFrontDistance(sr1c); }

int getFrontMiddleRaw() { return getDistanceRaw(sr1c); }

int getFrontLeft() { return getFrontDistance(sr2c); }

int getFrontLeftRaw() { return getDistanceRaw(sr2c); }

int getLeft() { return getLeftDistance(sr3c); }

int getLeftRaw() { return getDistanceRaw(sr3c); }

int getRightBack() { return getRightDistance(sr4c); }

int getRightBackRaw() { return getDistanceRaw(sr4c); }

int getRightFront() { return getRightDistance(sr5c); }

int getRightFrontRaw() { return getDistanceRaw(sr5c); }

void calibrateRaw() {
  Serial.print("rf:");
  Serial.print(getRightFront());
  Serial.print(" rb:");
  Serial.print(getRightBack());
  Serial.print(" l:");
  Serial.print(getLeft());
  Serial.print("\n");

  Serial.print("rf:");
  Serial.print(getRightFrontRaw());
  Serial.print(" rb:");
  Serial.print(getRightBackRaw());
  Serial.print(" l:");
  Serial.print(getLeftRaw());
  Serial.print("\n");
  Serial.print("\n");
  delay(1500);
}

// calibrate sensors
// calibration table of 20 value,
// 1st is for 0cm
// next is for 5cm, next for 10,.... up to 95cm
void calibrateSensors() {
  // callibrate front
  /*
  Serial.println("Front calibration, send g to begin");
  while (Serial.read() != 'g') {
  }

  sr0c.CalibrateStart();
  sr1c.CalibrateStart();
  sr2c.CalibrateStart();

  for (int i = 0; i < 19; i++) {
    sr0c.CalibrateNextStep();
    sr1c.CalibrateNextStep();
    sr2c.CalibrateNextStep();

    Serial.println("sr0 ");
    sr0c.DisplayCalibration(Serial);
    Serial.println("sr1 ");
    sr1c.DisplayCalibration(Serial);
    Serial.println("sr2");
    sr2c.DisplayCalibration(Serial);

    Serial.print("Move to ");
    Serial.print((i + 1) * 2);
    Serial.print(", and send g\n");
    while (Serial.read() != 'g') {
    }
    delay(1000);
  }

  Serial.println("End of calibration");

  sr0c.DisplayCalibration(Serial);
  sr1c.DisplayCalibration(Serial);
  sr2c.DisplayCalibration(Serial);

  // calibrate left side
  Serial.println("Left calibration, send g to begin");
  while (Serial.read() != 'g') {
  }

  sr3c.CalibrateStart();

  for (int i = 0; i < 19; i++) {
    sr3c.CalibrateNextStep();
    sr3c.DisplayCalibration(Serial);
    Serial.print("Move to ");
    Serial.print((i + 1) * 4);
    Serial.print(", and send g\n");
    while (Serial.read() != 'g') {
    }
    delay(1000);
  }

  Serial.println("End of calibration");
  sr3c.DisplayCalibration(Serial);
*/
  // calibrate right side
  Serial.println("Right calibration, send g to begin");
  while (Serial.read() != 'g') {
  }

  sr4c.CalibrateStart();
  //sr5c.CalibrateStart();

  for (int i = 0; i < 19; i++) {
    sr4c.CalibrateNextStep();
    sr4c.DisplayCalibration(Serial);
    //sr5c.CalibrateNextStep();
    //sr5c.DisplayCalibration(Serial);
    Serial.print("Move to ");
    Serial.print((i + 1) * 2);
    Serial.print(", and send g\n");
    while (Serial.read() != 'g') {
    }
    delay(1000);
  }

  Serial.println("End of calibration");
  sr4c.DisplayCalibration(Serial);
  //sr5c.DisplayCalibration(Serial);
}
