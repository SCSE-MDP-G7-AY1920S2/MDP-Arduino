#include "mdp_motor.h"
#include "mdp_sensor.h"

String toSend = "";
String command = "";

const int kMaxCalibrationTrial = 11;
bool DEBUG = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(50);
  startEncoder();
  setupPID();
  setupSensorsCalibration();
  // alignFront();
  // calibrateSensors();
  // calibrateStart();
}

void loop() {
  // put your main code here, to run repeatedly:
  String message = "";
  String com = "";
  if (Serial.available() > 0) {
    message = Serial.readString();

    switch (message.charAt(0)) {
      case 'K':  // send sensor values.
        sendSensor();
        break;

      case 'B':  // toggle debug mode.
        DEBUG = !DEBUG;
        break;

      case 'W':  // exploration move front.
        goForward();
        sendSensor();
        break;

      case 'S':  // exploration move back.
        goBackward();
        sendSensor();
        break;

      case 'A':  // exploration turn left.
        turnLeft(90);
        sendSensor();
        break;

      case 'D':  // exploration turn right.
        turnRight(90);
        sendSensor();
        break;

      case 'E':  // exploration end.
        sendFin();
        break;

      case 'F':  // calibrate front.
        calibrateFront();
        sendFin();
        break;

      case 'R':  // parallel wall.
        parallelWall();
        sendFin();
        break;

      case 'C':  // align robot using front and side walls.
        calibrateAll();
        sendFin();
        break;

      case 'Q':  // calibrate when facing east at the start.
        calibrateStart();
        sendFin();
        break;

      // Reset robot to face "North".
      case 'G':
        southToNorth();
        break;

      case 'H':
        eastToNorth();
        break;

      // Fast actions.
      case 'w':
      case 'a':
      case 's':
      case 'd':
        splitStringToAction(message);
        break;

      default:
        break;
    }
  }
}

// aligns the robot against the wall
void parallelWall() {
  // Right front further to obstacle by 1
  int rfOffset = 5;
  int rf = getRightFrontRaw() + rfOffset;
  int rb = getRightBackRaw();
  int diff = rf - rb;
  int trial = 0;
  startMotor();

  while (trial < kMaxCalibrationTrial && abs(diff) > 2) {
    if (rf < rb)
      turnLeftTicks(1);

    else if (rb < rf)
      turnRightTicks(1);

    rf = getRightFrontRaw() + rfOffset;
    rb = getRightBackRaw();
    diff = rf - rb;

    trial++;
  }
  endMotor();
}

// align robot to the front (angle)
void alignFront() {
  int flOffset = 0;
  int fr = getFrontRightRaw();
  int fl = getFrontLeftRaw() + flOffset;
  int diff = fr - fl;
  int trial = 0;

  startMotor();

  while (trial < kMaxCalibrationTrial && abs(diff) > 2) {
    if (fl > fr)
      turnRightTicks(1);

    else if (fr > fl)
      turnLeftTicks(1);

    fr = getFrontRightRaw();
    fl = getFrontLeftRaw() + flOffset;
    diff = fr - fl;

    trial++;
  }
  endMotor();
}

// align robot to the wall (distance)
void distanceFront() {
  int trial = 0;
  int dist = 115;
  startMotor();

  while (trial < kMaxCalibrationTrial && getFrontMiddleRaw() != dist) {
    if (getFrontRightRaw() < dist) goBackwardTicks(1);

    if (getFrontRightRaw() > dist) goForwardTicks(1);

    trial++;
  }
  endMotor();
}

// Turns robot back to "North" position
// when robot is facing "South".
void southToNorth() {
  alignFront();
  delay(100);
  distanceFront();
  delay(100);
  turnRight(90);
  delay(100);
  alignFront();
  delay(100);
  distanceFront();
  delay(100);
  alignFront();
  delay(100);
  turnRight(90);
}

// Turns robot back to "North" position
// when robot is facing "East".
// turn right
void eastToNorth() {
  alignFront();
  delay(100);
  distanceFront();
  delay(100);
  turnRight(90);
}

// Split fast actions string into separate actions.
void splitStringToAction(String com) {
  int j = 0;
  for (int i = 0; i < com.length(); i++) {
    if (com.charAt(i) == ',') {
      command = com.substring(j, i);
      j = i + 1;
      doFastAction(command, /*lastAction=*/false);
      delay(150);
    }
  }

  command = com.substring(j, com.length());
  doFastAction(command, /*lastAction=*/true);
}

void doFastAction(String com, bool lastAction) {
  if (com.charAt(0) == 'w') {
    int moveDistance = com.substring(1).toInt();
    if (lastAction) moveDistance--;
    if (moveDistance > 0 && moveDistance <= 15) {
      goForwardFast(moveDistance * 10);
    }
    if (lastAction) {
      if (moveDistance > 0) delay(150);
      maybeMoveOneGrid();
    }
  }

  else if (com.charAt(0) == 's') {
    int moveDistance = com.substring(1).toInt();
    if (moveDistance > 0 && moveDistance <= 15) {
      goBackwardFast(moveDistance * 10);
    }
  }

  else if (com.charAt(0) == 'a') {
    if (com.charAt(1) == '1')
      turnLeft(90);

    else if (com.charAt(1) == '2')
      turnLeft(180);
  }

  else if (com.charAt(0) == 'd') {
    if (com.charAt(1) == '1')
      turnRight(90);

    else if (com.charAt(1) == '2')
      turnRight(180);
  }
}

// For the last fast action, maybe move one grid
// depending on the front middle sensor reading.
void maybeMoveOneGrid() {
  int distFront = getFrontMiddleRaw();
  if (distFront > 200)
    goForwardFast(10);
  else if (distFront > 140)
    goForwardHalf();
}

// combines alignFront and distanceFront function
void calibrateFront() {
  alignFront();
  delay(100);
  distanceFront();
  delay(100);
}

// align against front and side wall
void calibrateAll() {
  parallelWall();
  delay(250);
  turnRight(90);
  delay(250);
  alignFront();
  delay(250);
  distanceFront();
  delay(250);
  turnLeft(90);
  delay(250);
  parallelWall();
}

// align against side and back wall
void calibrateStart() {
  turnRight(90);
  delay(100);
  alignFront();
  delay(100);
  distanceFront();
  delay(100);
  turnRight(90);
  delay(100);
  alignFront();
  delay(100);
  distanceFront();
  delay(100);
  turnLeft(90);
  delay(100);
  turnLeft(90);
  delay(100);
  parallelWall();
}

/* For checklist - need to do sharp/gentle turn
sharpAvoidance is for 90 degree turn
tiltAvoidance is for 45 degree turn*/
void sharpAvoidance() {
  while (true) {
    goForward();
    delay(300);
    if (getFrontMiddle() == 1) {
      delay(500);
      turnLeft(90);
      goForwardFast(20);
      delay(500);
      turnRight(90);
      delay(500);
      goForwardFast(50);
      turnRight(90);
      delay(500);
      goForwardFast(20);
      delay(500);
      turnLeft(90);
    }
  }
}

void tiltAvoidance() {
  while (true) {
    goForward();
    int dist = getFrontMiddleRaw();
    if (dist <= 30) {
      delay(500);
      turnLeft(45);
      goForwardFast(30);
      delay(500);
      turnRight(45);
      delay(500);
      goForwardFast(30);
      delay(500);
      turnRight(45);
      delay(500);
      goForwardFast(30);
      delay(500);
      turnLeft(45);
    }
  }
}

String getSensorRaw() {
  String rawJson = "{\"fr\":";
  rawJson.concat(getFrontRightRaw());

  rawJson.concat(",\"fl\":");
  rawJson.concat(getFrontLeftRaw());

  rawJson.concat(",\"fm\":");
  rawJson.concat(getFrontMiddleRaw());

  rawJson.concat(",\"left\":");
  rawJson.concat(getLeftRaw());

  rawJson.concat(",\"rf\":");
  rawJson.concat(getRightFrontRaw());

  rawJson.concat(",\"rb\":");
  rawJson.concat(getRightBackRaw());
  rawJson.concat("}");

  return rawJson;
}

// send sensor data
void sendSensor() {
  // delay(50);
  toSend = ";{\"from\":\"Arduino\",\"com\":\"SD\",\"fr\":";
  toSend.concat(getFrontRight());

  toSend.concat(",\"fl\":");
  toSend.concat(getFrontLeft());

  toSend.concat(",\"fm\":");
  toSend.concat(getFrontMiddle());

  toSend.concat(",\"left\":");
  toSend.concat(getLeft());

  toSend.concat(",\"rf\":");
  toSend.concat(getRightFront());

  toSend.concat(",\"rb\":");
  toSend.concat(getRightBack());

  if (DEBUG) {
    toSend.concat(",\"raw\":");
    toSend.concat(getSensorRaw());
  }

  toSend.concat("}");

  Serial.println(toSend);
  Serial.flush();
}

void sendFin() {
  toSend = ";{\"from\":\"Arduino\",\"com\":\"C\"}";

  Serial.println(toSend);
  Serial.flush();
}
