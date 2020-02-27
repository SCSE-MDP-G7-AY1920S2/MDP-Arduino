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
  bool isFront = false;

  if ((sensor.getIrPin() == s0) || (sensor.getIrPin() == s1) || (sensor.getIrPin() == s2))
    isFront = true;
  
  if ((isLongRange && dist > LR_MAX) || (isFront && dist > FRONT_MAX) || (!isLongRange && !isFront && dist > SR_MAX))
    return -1;

  if((isLongRange && dist < LR_MIN) || (isFront && dist < FRONT_MIN) || (!isLongRange && !isFront && dist < SR_MIN))
    return 1;
    
  //offset distance
  switch(sensor.getIrPin()){
    case s0: //fr
      dist -= 2;
      break;
    
    case s1: //fm
      dist -= 2;
      break;

    case s2: //fl
      dist -= 2;
      break;

    case s3: //l
      dist -= 15;
      break;

    case s4: //rb
      dist -= 5;
      break;

    case s5: //rf
      dist -= 5;
      break;
  }

  double grid = (dist / 10.0) + 1 + 0.5;
  return (int) grid;
}

// return distance from sensors (cm)
int getDistanceRaw(ZSharpIR& sensor) { return sensor.distance(); }
}  // namespace

void setupSensorsCalibration() {
  /*
  int tablesr0c[] = {4,   673, 484, 344, 281, 223, 190, 143, 139, 106,
                     96,  85,  75,  57,  42,  49,  41,  32,  27,  0};
  int tablesr1c[] = {2,   663, 489, 341, 268, 227, 197, 163, 145, 129,
                     114, 102, 90,  73,  70,  61,  53,  38,  35,  0};
  int tablesr2c[] = {4,   664, 481, 325, 254, 206, 191, 167, 127, 122,
                     107, 83,  67,  92,  70,  65,  59,  47,  40,  0};
  */

  int tablesr0c[]={3,457,617,618,520,440,374,327,293,263,244,224,209,192,178,170,159,151,147,0};
  int tablesr1c[]={2,355,616,618,536,452,384,337,297,271,249,231,212,184,188,162,161,115,165,0};
  int tablesr2c[]={3,346,620,620,524,439,371,324,292,263,240,218,202,186,175,167,160,146,142,0};
  
  int tablesr3c[] = {2,374,447,563,525,489,447,408,371,330,300,274,251,236,221,205,189,178,171,0};

  //int tablesr4c[]={3,618,620,618,517,437,377,330,293,270,246,228,208,196,184,169,165,158,151,0};
  //int tablesr5c[]={10,609,623,619,516,434,373,324,288,265,241,227,206,195,180,168,161,153,146,0};
  int tablesr4c[]={3,355,613,620,570,473,402,352,311,278,257,236,220,200,188,177,165,158,150,0};
  int tablesr5c[]={3,346,616,622,564,470,398,347,306,277,253,230,215,198,188,176,164,157,145,0};
  
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
*/
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
    Serial.print((i + 1) * 2);
    Serial.print(", and send g\n");
    while (Serial.read() != 'g') {
    }
    delay(1000);
  }

  Serial.println("End of calibration");
  sr4c.DisplayCalibration(Serial);
  sr5c.DisplayCalibration(Serial);
}
