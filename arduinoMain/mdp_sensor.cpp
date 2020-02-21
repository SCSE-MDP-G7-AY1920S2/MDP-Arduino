#include "mdp_sensor.h"

#include <Arduino.h>
#include <ZSharpIR.h>

namespace {
// short IR sensor
// c - calibrated
ZSharpIR sr0c(s0, SRmodel);
ZSharpIR sr1c(s1, SRmodel);
ZSharpIR sr2c(s2, SRmodel);
ZSharpIR sr4c(s4, SRmodel);
ZSharpIR sr5c(s5, SRmodel);

// long IR sensor
ZSharpIR sr3c = ZSharpIR(s3, LRmodel);

// return distance from sensors (grids)
int getDistance(ZSharpIR& sensor) {
  int dist = sensor.distance();
  bool isLongRange = sensor.getIrPin() == s3;
  if ((isLongRange && dist > LR_MAX) || (!isLongRange && dist > SR_MAX))
    return -1;

  //offset distance
  switch(sensor.getIrPin()){
    case s0: //fr
      dist -= 7;
      break;
    
    case s1: //fm
      dist -= 7;
      break;

    case s2: //fl
      dist -= 7;
      break;

    case s3: //l
      dist -= 15;
      break;

    case s4: //rb
      dist -= 8;
      break;

    case s5: //rf
      dist -= 8;
      break;
  }

  dist = (dist / 10) + 1;
  return isLongRange ? dist : dist;
}

// return distance from sensors (cm)
int getDistanceRaw(ZSharpIR& sensor) { return sensor.distance(); }
}  // namespace

void setupSensorsCalibration() {
  int tablesr0c[] = {4, 673, 484, 344, 281, 223, 190, 143, 139, 106, 96, 85, 75, 57, 42, 49, 41, 32, 27, 0};
  int tablesr1c[] = {2, 663, 489, 341, 268, 227, 197, 163, 145, 129, 114, 102, 90, 73, 70, 61, 53, 38, 35, 0};
  int tablesr2c[] = {4, 664, 481, 325, 254, 206, 191, 167, 127, 122, 107, 83, 67, 92, 70, 65, 59, 47, 40, 0};

  int tablesr3c[] = {2,   275, 486, 528, 482, 433, 382, 335, 293, 258,
                     235, 217, 197, 182, 170, 156, 148, 139, 139, 0};

  int tablesr4c[] = {3,   621, 491, 342, 262, 228, 216, 158, 112, 132,
                     128, 92,  108, 79,  76,  72,  60,  48,  64,  0};
  int tablesr5c[] = {3,   605, 494, 338, 260, 217, 185, 155, 132, 126,
                     112, 97,  132, 84,  89,  80,  68,  72,  60,  0};

  sr0c.ApplyCalibration(tablesr0c);
  sr1c.ApplyCalibration(tablesr1c);
  sr2c.ApplyCalibration(tablesr2c);
  sr3c.ApplyCalibration(tablesr3c);
  sr4c.ApplyCalibration(tablesr4c);
  sr5c.ApplyCalibration(tablesr5c);
}

int getFrontRight() { return getDistance(sr0c); }

int getFrontRightRaw() { return getDistanceRaw(sr0c); }

int getFrontMiddle() { return getDistance(sr1c); }

int getFrontMiddleRaw() { return getDistanceRaw(sr1c); }

int getFrontLeft() { return getDistance(sr2c); }

int getFrontLeftRaw() { return getDistanceRaw(sr2c); }

int getLeft() { return getDistance(sr3c); }

int getLeftRaw() { return getDistanceRaw(sr3c); }

int getRightBack() { return getDistance(sr4c); }

int getRightBackRaw() { return getDistanceRaw(sr4c); }

int getRightFront() { return getDistance(sr5c); }

int getRightFrontRaw() { return getDistanceRaw(sr5c); }

// calibrate sensors
// calibration table of 20 value,
// 1st is for 0cm
// next is for 5cm, next for 10,.... up to 95cm
void calibrateSensors() {
  // callibrate front
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
    Serial.print((i + 1) * 5);
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
    Serial.print((i + 1) * 5);
    Serial.print(", and send g\n");
    while (Serial.read() != 'g') {
    }
    delay(1000);
  }

  Serial.println("End of calibration");
  sr3c.DisplayCalibration(Serial);

  // calibrate right side
  Serial.println("Right calibration, send g to begin");
  while (Serial.read() != 'g') {
  }

  sr4c.CalibrateStart();
  sr5c.CalibrateStart();

  for (int i = 0; i < 19; i++) {
    sr4c.CalibrateNextStep();
    sr4c.DisplayCalibration(Serial);
    sr5c.CalibrateNextStep();
    sr5c.DisplayCalibration(Serial);
    Serial.print("Move to ");
    Serial.print((i + 1) * 5);
    Serial.print(", and send g\n");
    while (Serial.read() != 'g') {
    }
    delay(1000);
  }

  Serial.println("End of calibration");
  sr4c.DisplayCalibration(Serial);
  sr5c.DisplayCalibration(Serial);
}
