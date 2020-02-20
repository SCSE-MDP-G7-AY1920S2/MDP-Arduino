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

//return distance from sensors (cm).
int getDistance(ZSharpIR sensor, ZSharpIR sensorc){
  int dist;
  //multipler to set ratio for calibrated and non-calibrated value
  double multipler = 1;
  int lrMax = 60;
  int srMax = 30;

  boolean sr = true;
  if(sensor.getMax() != 800){
    sr = false;
  }

  dist = sensorc.distance();

  //if reading above/ below a certain range
  if(sr==false && dist > lrMax){
    dist = sensor.distance()/10;
  }
  
  else if(sr==true && dist > srMax){
    dist = sensor.distance()/10;
  }
  
  //otherwise use calibrated value
  else{
    dist = multipler*dist + (1-multipler)*(sensor.distance()/10);
  }
  
  if(sr==false){
    return (dist-3)/10;
  }
  if(sensorc.getIrPin() == sr4c.getIrPin()){
    return (dist+1)/10 + 1;
  }

  return (dist/10)+1;
}

int getDistanceRaw(ZSharpIR& sensor, ZSharpIR& sensorc) {
  int dist;
  // multipler to set ratio for calibrated and non-calibrated value
  double multipler = 1;
  int lrMax = 60;
  int srMax = 30;
  boolean sr = true;
  if (sensor.getMax() != 800) {
    sr = false;
  }
  dist = sensorc.distance();

  // if reading above/ below a certain range
  if (sr == false && dist > lrMax) {
    dist = sensor.distance() / 10;
  } else if (sr == true && dist > srMax) {
    dist = sensor.distance() / 10;
  } else {
    dist = multipler * dist + (1 - multipler) * (sensor.distance() / 10);
  }

  return dist;
}
}  // namespace

void setupSensorsCalibration() {
  int tablesr0c[] = {3,   623, 437, 313, 249, 176, 160, 140, 130, 129,
                     117, 86,  70,  50,  38,  30,  20,  15,  10,  0};
  int tablesr1c[] = {3,  619, 431, 308, 250, 200, 180, 165, 139, 119,
                     53, 56,  84,  89,  25,  21,  60,  80,  48,  0};
  int tablesr2c[] = {3,  621, 410, 295, 230, 190, 168, 145, 122, 118,
                     95, 82,  91,  95,  78,  59,  55,  54,  71,  0};
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

int getFrontRight() { return getDistance(sr0, sr0c); }

int getFrontRightRaw() { return getDistanceRaw(sr0, sr0c); }

int getFrontMiddle() { return getDistance(sr1, sr1c); }

int getFrontMiddleRaw() { return getDistanceRaw(sr1, sr1c); }

int getFrontLeft() { return getDistance(sr2, sr2c); }

int getFrontLeftRaw() { return getDistanceRaw(sr2, sr2c); }

int getLeft() { return getDistance(sr3, sr3c); }

int getRightBack() { return getDistance(sr4, sr4c); }

int getRightBackRaw() { return getDistanceRaw(sr4, sr4c); }

int getRightFront() { return getDistance(sr5, sr5c); }

int getRightFrontRaw() { return getDistanceRaw(sr5, sr5c); }

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
