#include "mdp_motor.h"
#include "mdp_sensor.h"

String toSend = "";
String command = "";

const int MAX_CALIBRATION_TRIAL = 20;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(50);
  startEncoder();
  setupPID();
  setupSensorsCalibration();
  //allignFront();
  //calibrateSensors();
  calibrateAll();
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  Serial.print(getLeftRaw());
  Serial.print(", ");
  Serial.print(getLeft());
  //Serial.print(", ");
  //Serial.print(getFrontRight());
  Serial.print("\n");
  delay(2000);
*/

  String message = "";
  String com = "";
  if (Serial.available() > 0) {
    message = Serial.readString();

    switch (message.charAt(0)) {
      case 'K':  // send sensor values.
        sendSensor();
        break;

      case 'W':  // exploration move front.
        goForward(10);
        sendSensor();
        break;

      case 'S':  // exploration move back.
        goBackward(10);
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

      case 'C':  // allign robot using front and side walls.
        calibrateAll();
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

  Serial.flush();
}

// aligns the robot against the wall
void parallelWall() {
  double rf = getRightFrontRaw();
  double rb = getRightBackRaw();
  double diff = rf - rb;
  int trial = 0;
  startMotor();

  while (trial < MAX_CALIBRATION_TRIAL && abs(diff) > 0.5) {
    if (rf < rb) turnLeft(1);

    else if (rb < rf) turnRight(1);

    rf = getRightFrontRaw();
    rb = getRightBackRaw();
    diff = rf - rb;

    trial++;
  }
  endMotor();
}

// align robot to the front (angle)
void allignFront() {
  int fr = getFrontRightRaw();
  int fl = getFrontLeftRaw();
  int diff = fr - fl;
  int trial = 0;

  startMotor();

  while (trial < MAX_CALIBRATION_TRIAL && abs(diff) != 0) {
    if (fl > fr) turnRight(1);

    else if (fr > fl)turnLeft(1);

    fr = getFrontRightRaw();
    fl = getFrontLeftRaw();
    diff = fr - fl;

    trial++;
  }
  endMotor();
}

// align robot to the wall (distance)
void distanceFront() {
  int trial = 0;
  int dist = 7;
  startMotor();

  while (trial < MAX_CALIBRATION_TRIAL && getFrontMiddleRaw() != dist) {
    if (getFrontRightRaw() < dist) goBackwardTicks(5);

    if (getFrontRightRaw() > dist) goForwardTicks(5);

    trial++;
  }
  endMotor();
}

// Turns robot back to "North" position
// when robot is facing "South".
void southToNorth() {
  delay(15000);
  turnLeft(90);
  parallelWall();
  distanceFront();
  allignFront();
  turnRight(90);
  distanceFront();
  allignFront();
  turnRight(90);
}

// Turns robot back to "North" position
// when robot is facing "East".
void eastToNorth() {
  delay(15000);
  parallelWall();
  distanceFront();
  allignFront();
  turnRight(90);
  distanceFront();
  allignFront();
  turnRight(90);
}

// Split fast actions string into separate actions.
void splitStringToAction(String com) {
  int j = 0;
  for (int i = 0; i < com.length(); i++) {
    if (com.charAt(i) == ',') {
      command = com.substring(j, i);
      j = i + 1;
      doFastAction(command);
      delay(500);
    }
  }

  command = com.substring(j, com.length());
  doFastAction(command);
}

void doFastAction(String com) {
  if (com.charAt(0) == 'w') {
    int moveDistance = com.substring(1).toInt();
    if (moveDistance > 0 && moveDistance <= 15) {
      goForwardFast(moveDistance * 10);
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

// combines allignFront and distanceFront function
void calibrateFront() {
  allignFront();
  delay(100);
  distanceFront();
  delay(100);
}

// allign against front and side wall
void calibrateAll() {
  parallelWall();
  delay(250);
  turnRight(90);
  delay(250);
  allignFront();
  delay(250);
  distanceFront();
  delay(250);
  turnLeft(90);
  parallelWall();
}

// send sensor data
void sendSensor() {
  delay(500);
  toSend = ";{\"from\":\"Arduino\",\"com\":\"SD\",\"fr\":";
  toSend.concat(getFrontRight());

  toSend.concat(",\"fl\":");
  toSend.concat(getFrontLeft());

  toSend.concat(",\"fm\":");
  toSend.concat(getFrontMiddle());

  toSend.concat(",\"left\":");
  toSend.concat(getLeft());
  // toSend.concat("-1");

  toSend.concat(",\"rf\":");
  toSend.concat(getRightFront());
  // toSend.concat("-1");

  toSend.concat(",\"rb\":");
  toSend.concat(getRightBack());
  // toSend.concat("-1");
  toSend.concat("}");

  Serial.println(toSend);
  Serial.flush();
}

void sendFin() {
  toSend = ";{\"from\":\"Arduino\",\"com\":\"C\"}";

  Serial.println(toSend);
  Serial.flush();
}
