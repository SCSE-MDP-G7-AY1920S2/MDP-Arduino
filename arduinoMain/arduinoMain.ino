#include "mdp_motor.h"
#include "mdp_sensor.h"

String toSend = "";
String command = "";

const int kMaxCalibrationTrial = 15;
const int kMaxCalibrationTrialInit = 30;
const int kMaxCalibrationTrialTurn = 3;
bool initCalibration = false;
bool DEBUG = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(50);
  startEncoder();
  setupPID();
  setupSensorsCalibration();
}

void loop() {
  // put your main code here, to run repeatedly:
  String message = "";
  String com = "";
  if (Serial.available() > 0) {
    message = Serial.readString();

    switch (message.charAt(0)) {
      case 'K':  // send sensor values.
        sendSensor("SD");
        break;

      case 'B':  // toggle debug mode.
        DEBUG = !DEBUG;
        break;

      case 'W':  // exploration move front.
        goForward();
        sendSensor("MF");
        break;

      case 'S':  // exploration move back.
        goBackward();
        sendSensor("MF");
        break;

      case 'A':  // exploration turn left.
        turnLeft(90);
        sendSensor("MF");
        break;

      case 'D':  // exploration turn right.
        turnRight(90);
        sendSensor("MF");
        break;

      case 'E':  // exploration end.
        sendFin();
        break;

      case 'F':  // calibrate front for wall.
        calibrateFront(/*isBlock=*/false);
        sendFin();
        break;

      case 'f':  // calibrate front for block.
        calibrateFront(/*isBlock=*/true);
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
        delay(500);
        initCalibration = true;
        turnRight(90);
        adjustTurnTicks();
        turnLeft(90);
        calibrateStart();
        initCalibration = false;
        sendFin();
        break;

      case 'G': // robot come back facing south
        initCalibration = true;
        adjustTurnTicks();
        southToNorth();
        initCalibration = false;
        sendFin();
        break;

      case 'H': // robot come back facing west
        initCalibration = true;
        adjustTurnTicks();
        turnLeft(90);
        southToNorth();
        initCalibration = false;
        sendFin();
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
  int rfOffset = -4;
  int rf = getRightFrontRaw() + rfOffset;
  int rb = getRightBackRaw();
  int diff = rf - rb;
  int trial = 0;
  int maxTrial =
      initCalibration ? kMaxCalibrationTrialInit : kMaxCalibrationTrial;
  startMotor();

  while (trial < maxTrial && abs(diff) > 0) {
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
  int frOffset = 0;
  int fr = getFrontRightRaw() + frOffset;
  int fl = getFrontLeftRaw();
  int diff = fr - fl;
  int trial = 0;
  int maxTrial =
      initCalibration ? kMaxCalibrationTrialInit : kMaxCalibrationTrial;

  startMotor();

  while (trial < maxTrial && abs(diff) > 0) {
    if (fl > fr)
      turnRightTicks(1);

    else if (fr > fl)
      turnLeftTicks(1);

    fr = getFrontRightRaw() + frOffset;
    fl = getFrontLeftRaw();
    diff = fr - fl;

    trial++;
  }
  endMotor();
}

// align robot to the wall (distance)
void distanceFront(bool isBlock) {
  int trial = 0;
  int dist = isBlock ? 115 : 114;
  int maxTrial =
      initCalibration ? kMaxCalibrationTrialInit : kMaxCalibrationTrial;
  startMotor();

  int fm = getFrontMiddleRaw();
  while (trial < maxTrial && abs(fm - dist) > 1) {
    if (fm < dist) goBackwardTicks(2);

    if (fm > dist) goForwardTicks(2);
    fm = getFrontMiddleRaw();

    trial++;
  }
  endMotor();
}

// Adjust the turn ticks based on fl and fr sensor readings.
// This function assusmes that the robot is facing a wall.
void adjustTurnTicks() {
  int fl, fr;
  int trial = 0;
  calibrateFront(/*isBlock=*/false);
  do {
    for(int i = 0; i < 4; i++) {
      turnLeft(90);
      delay(100);
    }
    delay(300);
    fl = getFrontLeftRaw();
    fr = getFrontRightRaw();
    calibrateFront(/*isBlock=*/false);

    if(abs(fl - fr) <= 1)
      break;
    adjutstTurnLeftTicks(fr - fl);
    trial++;
  } while (trial < kMaxCalibrationTrialTurn);

  trial = 0;

  do {
    for (int i = 0; i < 4; i++) {
      turnRight(90);
      delay(100);
    }
    delay(300);
    fl = getFrontLeftRaw();
    fr = getFrontRightRaw();
    calibrateFront(/*isBlock=*/false);

    if(abs(fl - fr) <= 1)
      break;
    adjutstTurnRightTicks(fl - fr);
    trial++;
  } while (trial < kMaxCalibrationTrialTurn);
}

// turn 180
void southToNorth() {
  calibrateFront(/*isBlock=*/false);
  delay(100);
  turnRight(90);
  delay(100);
  calibrateFront(/*isBlock=*/false);
  delay(100);
  turnRight(90);
}

// turn right 90
void eastToNorth() {
  calibrateFront(/*isBlock=*/false);
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
void calibrateFront(bool isBlock) {
  distanceFront(isBlock);
  delay(100);
  alignFront();
  delay(100);
}

// align against front and side wall
void calibrateAll() {
  calibrateFront(/*isBlock=*/false);
  delay(100);
  turnRight(90);
  delay(100);
  calibrateFront(/*isBlock=*/false);
  delay(100);
  turnLeft(90);
  delay(100);
  parallelWall();
}

// align against side and back wall
void calibrateStart() {
  turnRight(90);
  delay(100);
  alignFront();
  delay(100);
  distanceFront(/*isBlock=*/false);
  delay(100);
  turnRight(90);
  delay(100);
  alignFront();
  delay(100);
  distanceFront(/*isBlock=*/false);
  delay(100);
  turnLeft(90);
  delay(100);
  distanceFront(/*isBlock=*/false);
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
void sendSensor(String comType) {
  toSend = ";{\"from\":\"Arduino\",\"com\":\"" + comType + "\"";
  toSend.concat(",\"fr\":");
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
  // delay(50);
  toSend = ";{\"from\":\"Arduino\",\"com\":\"C\"}";

  Serial.println(toSend);
  Serial.flush();
}
